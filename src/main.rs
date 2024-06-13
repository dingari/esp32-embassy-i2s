#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    dma::{Dma, DmaPriority},
    dma_circular_buffers,
    gpio::Io,
    i2s::{asynch::*, DataFormat, I2s, Standard},
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
    timer::timg::TimerGroup,
};

use num_traits::real::Real;
use core::f32::consts::PI;

const SAMPLE_RATE: f32 = 44100.0;
const NUM_CHANNELS: usize = 2;
const NUM_SAMPLES: usize = 4096;

#[main]
async fn main(_spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    log::info!("Init!");

    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timg0 = TimerGroup::new_async(peripherals.TIMG0, &clocks);
    esp_hal_embassy::init(&clocks, timg0);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.channel0;

    let (tx_buffer, mut tx_descriptors, _, mut rx_descriptors) = dma_circular_buffers!(NUM_SAMPLES * NUM_CHANNELS * core::mem::size_of::<i16>(), 0);

    let i2s = I2s::new(
        peripherals.I2S0,
        Standard::Philips,
        DataFormat::Data16Channel16,
        (SAMPLE_RATE as u32).Hz(),
        dma_channel.configure_for_async(
            false,
            &mut tx_descriptors,
            &mut rx_descriptors,
            DmaPriority::Priority0,
        ),
        &clocks,
    );

    let i2s_tx = i2s
        .i2s_tx
        .with_bclk(io.pins.gpio2)
        .with_ws(io.pins.gpio4)
        .with_dout(io.pins.gpio5)
        .build();

    let sine_sample = |t: f32, f: f32, sr: f32| {
        let smpl_f32 = ((2.0 * PI * f * t / sr) as f32).sin();
        let smpl_i16 = (smpl_f32 * i16::MAX as f32) as i16;

        smpl_i16
    };

    let sample_fn = |t: f32| {
        let single_period = NUM_SAMPLES * NUM_CHANNELS / 2;
        let freq = (1.0 / single_period as f32) * SAMPLE_RATE;

        sine_sample(t, freq, SAMPLE_RATE)
    };

    const N: usize = 4;
    let mut filler = [0i16; NUM_SAMPLES / N * NUM_CHANNELS];
    let filler_bytes = unsafe { core::slice::from_raw_parts(&filler as *const _ as *const u8, filler.len() * 2) };

    let mut t = 0_f32;

    log::info!("DMA buffer: {} bytes, filler: {} channel samples ({} bytes)", tx_buffer.len(), filler.len() / NUM_CHANNELS, filler.len() * core::mem::size_of::<i16>());
    let mut transaction = i2s_tx.write_dma_circular_async(tx_buffer).unwrap();
    for j in 0..4 {
        let step = match j % N {
            j if (0..N / 2).contains(&j) => j as f32 * 2.0 / N as f32 - 1.0,
            j if (N / 2..N).contains(&j) => (j + 1) as f32 * 2.0 / N as f32 - 1.0,
            _ => 0.0
        };

        for i in (0..filler.len()).step_by(NUM_CHANNELS) {
            let smpl_i16 = sample_fn(t + (i / NUM_CHANNELS) as f32);
            let (left, _right) = (smpl_i16, -smpl_i16);

            filler[i] = left;
            filler[i + 1] = (i16::MAX as f32 * step) as i16;
        }

        let avail = transaction.available().await.unwrap_or(0);
        let bytes_written = transaction.push(&filler_bytes).await.unwrap();
        t += (bytes_written / NUM_CHANNELS / core::mem::size_of::<i16>()) as f32;
        log::info!("written bytes: {bytes_written} (available: {avail}) t: {t}");
    }
}
