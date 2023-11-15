# Edge Impulse firmware for nRF52840 DK / nRF5340 DK

[Edge Impulse](https://www.edgeimpulse.com) enables developers to create the next generation of intelligent device solutions with embedded Machine Learning. This repository contains the Edge Impulse firmware for the Nordic Semiconductor nRF52840 DK / nRF5340 DK development boards, in combination with the ST X_NUCLEO-IKS02A1 shield. This combination supports all Edge Impulse device features, including ingestion, remote management and inferencing.

> **Note:** Do you just want to use this development board with Edge Impulse? No need to build this firmware. See the instructions for the [nRF52840 DK](https://docs.edgeimpulse.com/docs/development-platforms/officially-supported-mcu-targets/nordic-semi-nrf52840-dk) and the [nRF5340 DK](https://docs.edgeimpulse.com/docs/development-platforms/officially-supported-mcu-targets/nordic-semi-nrf5340-dk) for prebuilt images and instructions, or use the [data forwarder](https://docs.edgeimpulse.com/docs/tools/edge-impulse-cli/cli-data-forwarder) to capture data from any sensor.

## Building the device firmware (locally)

1. Install the [nRF Connect SDK](https://docs.nordicsemi.com/bundle/ncs-2.4.0/page/nrf/getting_started/installing.html) in a *separate* folder from this repository (e.g. `~/repos/ncs`).

2. Clone this repository:

    ```bash
    $ git clone https://github.com/edgeimpulse/firmware-nordic-nrf52840dk-nrf5340dk
    ```

3. Build the application:

    **nRF52840 DK**

    ```bash
    $ west build -b nrf52840dk_nrf52840
    ```

    **nRF5340 DK**

    ```bash
    $ west build -b nrf5340dk_nrf5340_cpuapp
    ```

## Building the device firmware (Docker)

1. Clone this repository:

    ```bash
    $ git clone https://github.com/edgeimpulse/firmware-nordic-nrf52840dk-nrf5340dk
    ```

2. Build the Docker container:

    ```bash
    $ docker build -t edge-impulse-nordic .
    ```

3. Build the application:


    **nRF52840 DK**

    ```bash
    $ docker run --rm -v $PWD:/app edge-impulse-nordic west build -b nrf52840dk_nrf52840
    ```

    **nRF5340 DK**

    ```bash
    $ docker run --rm -v $PWD:/app edge-impulse-nordic west build -b nrf5340dk_nrf5340_cpuapp
    ```

## Flashing (JLink Mass Storage)

1. Connect the board and power on.
2. Copy `build/zephyr/zephyr.bin` to the `JLINK` mass storage device.

## Flashing (command line)

1. Connect the board and power on.
2. Flash the board controller firmware:

    ```bash
    $ cd board-controller/
    $ west flash
    ```

## Working with other sensors

You can easily add support for other accelerometers, PDM microphones or even completely different sensors to this firmware through either built-in Zephyr drivers, or through the Zephyr sensor API. Here's an example of adding the IIS2DLPC accelerometer through the sensor API:

1. Connect to the sensor using the device tree through an overlay file, and add the sensor definition in the project configuration.

    **nrf52840dk_nrf52840.overlay**

    ```
    &i2c0 {
        clock-frequency = <I2C_BITRATE_FAST>;
        iis2dlpc@19 {
            compatible = "st,iis2dlpc";
            reg = <0x19>;
            drdy-gpios =  <&gpio0 30 GPIO_ACTIVE_HIGH>; /* A4 - INT2 */
            label = "IIS2DLPC";
        };
    };
    ```

    **prj.conf**

    ```
    CONFIG_SENSOR=y
    CONFIG_IIS2DLPC=y
    CONFIG_IIS2DLPC_ACCEL_RANGE_2G=y
    CONFIG_IIS2DLPC_ODR_1600=y
    CONFIG_IIS2DLPC_POWER_MODE=4
    ```

2. Update the source code to reference the new sensor. The IMU sensor source is located in [ei_inertialsensor.cpp](edge-impulse/ingestion-sdk-platform/NordicSemi-nrf52/sensors/ei_inertialsensor).

    Declare any objects required by the sensor:

    ```c
    const struct device *accel;
    struct sensor_value accel_x;
    struct sensor_value accel_y;
    struct sensor_value accel_z;
    ```

    Then, initialize the sensor in the `bool ei_inertial_init()` function:

    ```c
    accel = DEVICE_DT_GET("IIS2DLPC");
    if(accel == NULL){
        ei_printf("No device IIS2DLPC found; did initialization fail\n");
    }
    else{
        ei_printf("Found device IIS2DLPC\n");
    }
    ```

    And, finally, read from the sensor in the `void ei_inertial_read_data(void)` function:

    ```c
    if(accel) {
        sensor_sample_fetch(accel);
        sensor_channel_get(accel, SENSOR_CHAN_ACCEL_X, &accel_x);
        sensor_channel_get(accel, SENSOR_CHAN_ACCEL_Y, &accel_y);
        sensor_channel_get(accel, SENSOR_CHAN_ACCEL_Z, &accel_z);
        acceleration_g[0] = sensor_value_to_double(&accel_x);
        acceleration_g[1] = sensor_value_to_double(&accel_y);
        acceleration_g[2] = sensor_value_to_double(&accel_z);

        cb_sampler((const void *)&acceleration_g[0], SIZEOF_N_AXIS_SAMPLED);
        k_usleep(sample_interval_real_us);
    }
    ```

    Last, delete all code related to the old sensor driver.

3. Compile and flash the application.

You can find more information in the [Zephyr RTOS sensors API](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.7.0/zephyr/reference/peripherals/sensor.html) documentation, and the [Sensor samples](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.7.0/zephyr/samples/sensor/sensor.html).
