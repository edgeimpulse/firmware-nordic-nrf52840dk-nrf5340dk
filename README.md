# Edge Impulse firmware for nRF52840 DK / nRF5340 DK

[Edge Impulse](https://www.edgeimpulse.com) enables developers to create the next generation of intelligent device solutions with embedded Machine Learning. This repository contains the Edge Impulse firmware for the Nordic Semiconductor nRF52840 DK / nRF5340 DK development boards, in combination with the ST IKS02A shield. This combination supports all Edge Impulse device features, including ingestion, remote management and inferencing.

> **Note:** Do you just want to use this development board with Edge Impulse? No need to build this firmware. See the instructions for the [nRF52840 DK](https://docs.edgeimpulse.com/docs/nordic-semi-nrf52840-dk) and the [nRF5340 DK](https://docs.edgeimpulse.com/docs/nordic-semi-nrf5340-dk) for prebuilt images and instructions, or use the [data forwarder](https://docs.edgeimpulse.com/docs/cli-data-forwarder) to capture data from any sensor.

## Requirements

**Hardware**

* Nordic Semiconductor [nRF52840 DK](https://docs.edgeimpulse.com/docs/nordic-semi-nrf52840-dk) or [nRF5340 DK](https://docs.edgeimpulse.com/docs/nordic-semi-nrf5340-dk) development board.
* [X-NUCLEO-IKS02A1](https://www.st.com/en/ecosystems/x-nucleo-iks02a1.html) shield.

    > No IKS02A1 shield? You can modify this firmware relatively easily to work with other accelerometers or PDM microphones that are supported in Zephyr. See [Working with other sensors](#working-with-other-sensors).

**Software**

* [nRF Connect SDK v1.9.1](https://www.nordicsemi.com/Software-and-tools/Software/nRF-Connect-SDK)
* [GNU ARM Embedded Toolchain 9-2019-q4-major](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads).
* [nRF Command Line Tools](https://www.nordicsemi.com/Software-and-tools/Development-Tools/nRF-Command-Line-Tools/Download).

Or you can build this application with Docker (see below).

## Building the device firmware (locally)

1. Install and configure the nRF Connect SDK:
    1. [nRF Connect SDK](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/gs_installing.html) in a *separate* folder from this repository (e.g. `~/repos/ncs`).
    1. Check out NCS version 1.9.1:

        ```
        $ cd ~/repos/ncs/nrf
        $ git checkout v1.9.1
        $ cd ..
        $ west update
        ```

    1. Set your `ZEPHYR_BASE` environment variable to `~/repos/ncs/zephyr`.

1. Clone this repository:

    ```
    $ git clone https://github.com/edgeimpulse/firmware-nrf52840-5340
    ```

1. Build the application:

    **nRF52840 DK**

    ```
    $ west build -b nrf52840dk_nrf52840
    ```

    **nRF5340 DK**

    ```
    $ west build -b nrf5340dk_nrf5340_cpuapp
    ```

1. Flash the application:

    ```
    $ west flash
    ```

## Building the device firmware (Docker)

1. Clone this repository:

    ```
    $ git clone https://github.com/edgeimpulse/firmware-nrf52840-5340
    ```

1. Build the Docker container:

    ```
    $ docker build -t edge-impulse-nordic .
    ```

1. Build the application:


    **nRF52840 DK**

    ```
    $ docker run --rm -v $PWD:/app edge-impulse-nordic west build -b nrf52840dk_nrf52840
    ```

    **nRF5340 DK**

    ```
    $ docker run --rm -v $PWD:/app edge-impulse-nordic west build -b nrf5340dk_nrf5340_cpuapp
    ```

1. Copy `build/zephyr/zephyr.bin` to the `JLINK` mass storage device.

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
    accel = device_get_binding("IIS2DLPC");
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
