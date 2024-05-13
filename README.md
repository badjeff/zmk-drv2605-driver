# DRV2605 driver implementation for ZMK

This module exposes DRV2605 internal interrupt trigger instruction set via Zephyr's `sensor_driver_api`.

Example is available at [here](https://github.com/badjeff/zmk-output-behavior-listener/blob/54ba63badb1f5bf9697b2e14753396eb54b153c7/src/output_haptic_feedback.c#L51) in [zmk-output-behavior-listener](https://github.com/badjeff/zmk-output-behavior-listener).

## Installation

Include this project on ZMK's west manifest in `config/west.yml`:

```yml
manifest:
  remotes:
    #...
    - name: badjeff
      url-base: https://github.com/badjeff
    #...
  projects:
    #...
    - name: zmk-drv2605-driver
      remote: badjeff
      revision: main
    #...
  self:
    path: config
```

Update `board.overlay` adding the necessary bits (update the pins for your board accordingly):

```dts
&pinctrl {
    i2c0_default: i2c0_default {
        group1 {
            psels = <NRF_PSEL(TWIM_SDA, 0, 9)>,
                    <NRF_PSEL(TWIM_SCL, 0, 10)>;
        };
    };

    i2c0_sleep: i2c0_sleep {
        group1 {
            psels = <NRF_PSEL(TWIM_SDA, 0, 9)>,
                    <NRF_PSEL(TWIM_SCL, 0, 10)>;
            low-power-enable;
        };
    };
};

&i2c0 {
    status = "okay";
    compatible = "nordic,nrf-twim";
    pinctrl-0 = <&i2c0_default>;
    pinctrl-1 = <&i2c0_sleep>;
    pinctrl-names = "default", "sleep";

    drv2605_0: drv2605@5a {
        compatible = "ti,drv2605";
        reg = <0x5a>;
        library = <6>;
    };
};
```

Enable the driver config in `<shield>.config` file (read the Kconfig file to find out all possible options):

```conf
CONFIG_I2C=y
CONFIG_DRV2605=y
# CONFIG_DRV2605_LOG_LEVEL_DBG=y
```
