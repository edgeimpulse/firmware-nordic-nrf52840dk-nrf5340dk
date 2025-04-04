/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef EI_DEVICE_INFO
#define EI_DEVICE_INFO

/* Include ----------------------------------------------------------------- */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>


// Available sensors to sample from on this board
typedef struct {
    // Name (e.g. 'Built-in accelerometer')
    const char *name;
    // Frequency list
    float frequencies[5];
    // Max. sample length in seconds (could be depending on the size of the flash chip)
    uint16_t max_sample_length_s;
    // Start sampling, this function should be blocking and is called when sampling commences
// #ifdef __MBED__
//     Callback<bool()> start_sampling_cb;
// #else
    bool (*start_sampling_cb)();
//#endif
} ei_device_sensor_t;

typedef struct {
    size_t width;
    size_t height;
    uint8_t color_depth;
} ei_device_snapshot_resolutions_t;

typedef struct {
    size_t width;
    size_t height;
} ei_device_resize_resolutions_t;

typedef struct {
    char str[32];
    int val;
} ei_device_data_output_baudrate_t;

/**
 * @brief      Default class for device characteristics
 */
class EiDeviceInfo
{
private:
	/** Default device ID & type */
	const char *ei_dev_default_type = "Default type";
	const char *ei_dev_default_id = "1:2:3:4:5:6";


public:
	EiDeviceInfo(void){};
	~EiDeviceInfo(void){};

	/**
	 * @brief      Gets the device ID string
	 *
	 * @param      out_buffer  Destination buffer for ID
	 * @param      out_size    Length of ID in bytes
	 *
	 * @return     Zero if ok, non-zero to signal an error
	 */
	virtual int get_id(uint8_t out_buffer[32], size_t *out_size)
	{
		int length = strlen(ei_dev_default_id) + 1;
		memcpy(out_buffer, ei_dev_default_id, length);
		*(out_size) = length;
		return 0;
	};

	/**
	 * @brief      Get pointer to zero terminatied id string
	 *
	 * @return     The id pointer.
	 */
	virtual const char *get_id_pointer(void)
	{
		return ei_dev_default_id;
	}

	/**
	 * @brief      Gets the device type string
	 *
	 * @param      out_buffer  Destination buffer for type
	 * @param      out_size    Length of type string in bytes
	 *
	 * @return     Zero if ok, non-zero to signal an error
	 */
	virtual int get_type(uint8_t out_buffer[32], size_t *out_size)
	{
		int length = strlen(ei_dev_default_type) + 1;
		memcpy(out_buffer, ei_dev_default_type, length);
		*(out_size) = length;
		return 0;
	}

	/**
	 * @brief      Get pointer to zero terminatied type string
	 *
	 * @return     The type pointer.
	 */
	virtual const char *get_type_pointer(void)
	{
		return ei_dev_default_type;
	}

	/**
	 * @brief      Gets the wifi connection status.
	 *
	 * @return     The wifi connection status.
	 */
	virtual bool get_wifi_connection_status(void)
	{
		return false;
	};

	/**
	 * @brief      Gets the wifi present status.
	 *
	 * @return     The wifi present status.
	 */
	virtual bool get_wifi_present_status(void)
	{
		return false;
	};

	/**
	 * @brief      Get pointer to the list of available sensors, and the number of sensors
	 *             used
	 * @param      sensor_list       Place pointer to sensor list here
	 * @param      sensor_list_size  Fill in the number of sensors in the list
	 *
	 * @return     The sensor list.
	 */
	virtual bool get_sensor_list(const ei_device_sensor_t **sensor_list, size_t *sensor_list_size)
	{
		*sensor_list 	  = NULL;
		*sensor_list_size = 0;
		return true;
	}

    /**
	 * @brief      Create resolution list for snapshot setting
	 *             The studio and daemon require this list
	 * @param      snapshot_list       Place pointer to resolution list
	 * @param      snapshot_list_size  Write number of resolutions here
	 *
	 * @return     False if all went ok
	 */
    virtual bool get_snapshot_list(
        const ei_device_snapshot_resolutions_t **snapshot_list,
        size_t *snapshot_list_size,
        const char **color_depth)
    {
        return true;
    }

    virtual void set_default_data_output_baudrate()
	{

	}

    virtual void set_max_data_output_baudrate()
	{

	}
};


#endif
