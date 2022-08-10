/* Edge Impulse ingestion SDK
 * Copyright (c) 2020 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef EI_INERTIAL_SENSOR
#define EI_INERTIAL_SENSOR

/* Include ----------------------------------------------------------------- */
#include "ei_sampler.h"
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/sensor.h>

/** Number of axis used and sample data format */
typedef float sample_format_t;
#define N_AXIS_SAMPLED          3 
#define SIZEOF_N_AXIS_SAMPLED   (sizeof(sample_format_t) * N_AXIS_SAMPLED)


/* Function prototypes ----------------------------------------------------- */
bool ei_inertial_init(void);
int ei_inertial_read_data(void);
bool ei_inertial_sample_start(sampler_callback callback, float sample_interval_ms);
bool ei_inertial_setup_data_sampling(void);
void ei_intertial_read_data_one(void);

#endif
