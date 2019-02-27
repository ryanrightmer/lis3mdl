# `lis3mdl`

A platform agnostic driver to interface with the LIS3MDL (3-axis magnetic sensor).

# Examples

Using the [B-L475E-IOT01A1](https://www.st.com/en/evaluation-tools/b-l475e-iot01a.html) board and the [`stm32l4xx_hal`](https://crates.io/crates/stm32l4xx-hal) crate.

```rust
// Set up the L475 peripherals
let p = stm32l4xx_hal::stm32l4::stm32l4x5::Peripherals::take().unwrap();
let mut rcc = p.RCC.constrain();
let mut gpiob = p.GPIOB.split(&mut rcc.ahb2);

// Set up clock signal as open drain with internal pull up on PB10 in alternate mode 4
let mut scl = gpiob.pb10.into_open_drain_output(&mut gpiob.moder, &mut gpiob.otyper);
scl.internal_pull_up(&mut gpiob.pupdr, true);
let scl = scl.into_af4(&mut gpiob.moder, &mut gpiob.afrh);

// Set up the data signal as open drain with internal pull up on PB11 in alternate mode 4
let mut sda = gpiob.pb11.into_open_drain_output(&mut gpiob.moder, &mut gpiob.otyper);
sda.internal_pull_up(&mut gpiob.pupdr, true);
let sda = sda.into_af4(&mut gpiob.moder, &mut gpiob.afrh);

// Set up the I2C peripheral (using I2C2)
let i2c2 = I2c::i2c2(p.I2C2,(scl,sda),100.khz(),clocks,&mut rcc.apb1r1);

// Initialize the LIS3MDL with the I2C
let mut lis3mdl_device = Lis3mdl::new(i2c2).unwrap();

// Read the X, Y, and Z axes values in milliGauss
let xyz = lis3mdl_device.get_mag_axes_mgauss().unwrap();
```

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.

