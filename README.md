# korz-joystick

ZMK module: an analog joystick (two ADC axes and a push button) as a pointing device. A Zephyr input
driver (`korz,joystick`) that emits relative X/Y (or wheel) events and a button; a `zmk,input-listener`
in your shield turns them into mouse reports.

```
joystick: joystick {
    compatible = "korz,joystick";
    io-channels = <&adc 1>, <&adc 2>;         /* X then Y; NOT channel 0 (ZMK's battery driver uses it) */
    button-gpios = <&gpio0 31 (GPIO_ACTIVE_HIGH | GPIO_PULL_DOWN)>;
};
joystick_listener { compatible = "zmk,input-listener"; device = <&joystick>; };
```

nRF52840 analog pins: P0.02-P0.05 and P0.28-P0.31. Settings (`KORZ_JOYSTICK`, `KORZ_JOY_*`): mouse or
scroll mode, speed, dead zone, ADC range, sampling period, debug log. The centre is measured at start-up
(25 samples after 1 s): keep the stick still when plugging in.
