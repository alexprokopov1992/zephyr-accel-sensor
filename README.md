# Датчик руху та датчик піддомкрачування.

Розроблен для використання в автомобільних системах.

Створіть запис у описі плати запис.

```devicetree
/ {
    aliases {
        accelerometer = &mma8652fc;
    };

	accel_sensor: accel_sensor {
		compatible = "baden,accel-sensor";
		status = "okay";
        accelerometer = &mma8652fc;
	};
};

&i2c1 {
	status = "okay";
	pinctrl-0 = <&i2c1_pa7_pa6>;
	pinctrl-names = "default";

	// Int on PC6/INT2
	mma8652fc: mma8652fc@1d {
		compatible = "nxp,fxos8700","nxp,mma8652fc";
		reg = <0x1d>;
		int1-gpios = <&gpioc 6 (GPIO_ACTIVE_LOW | GPIO_PULL_UP)>;
		// interrupt-parent = <&gpioc>;
		// interrupts = <6 IRQ_TYPE_EDGE_FALLING>;
	};
};
```
