# LIS2DH

Driver crate for the ST [LIS2DH12] accelerometer.
Compatible with [embedded-hal] and [embedded-hal-async] traits.

# Example usage

```rust
let mut accelerometer = Lis2dh::new(i2c, Sa0Pad::High);

accelerometer.set_mode(Mode::Normal).await.unwrap();
accelerometer.set_output_data_rate(OutputDataRate::Hz100).await.unwrap();

let int1_config = Int1Config::FifoWatermark;
accelerometer.configure_int1(&int1_config).await.unwrap();

accelerometer.configure_fifo(FifoConfig::Stream { watermark: 9 }).await.unwrap();
accelerometer.enable_fifo(true).await.unwrap();

let mut data = [AccelerationData::default(); 10];
loop {
    // Wait until the accelerometer fills the FIFO
    accelerometer_int1.wait_for_high().await;
    accelerometer.read_data(&mut data).await.unwrap();
}
```

# Resources

- [Product Page]
- [Datasheet]
- [Application Note]

[LIS2DH12]: https://www.st.com/en/mems-and-sensors/lis2dh12.html
[embedded-hal]: https://docs.rs/embedded-hal/latest/embedded_hal/
[embedded-hal-async]: https://docs.rs/embedded-hal-async/latest/embedded_hal_async/
[Datasheet]: https://www.st.com/resource/en/datasheet/lis2dh12.pdf
[Product Page]: https://www.st.com/en/mems-and-sensors/lis2dh12.html
[Application Note]: https://www.st.com/resource/en/application_note/an5005-lis2dh12-ultralowpower-highperformance-3axis-nano-accelerometer-with-digital-output-stmicroelectronics.pdf
