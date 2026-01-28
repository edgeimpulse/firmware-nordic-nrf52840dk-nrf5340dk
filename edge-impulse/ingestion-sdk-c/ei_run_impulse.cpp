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

/* Include ----------------------------------------------------------------- */
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "edge-impulse-sdk/classifier/ei_print_results.h"
#include "edge-impulse-sdk/dsp/numpy.hpp"
#include "ei_inertialsensor.h"
#include "ei_microphone.h"
#include "ei_device_nordic_nrf52.h"
#include "ble_nus.h"

static char ble_printf[64] = {0};

#if defined(EI_CLASSIFIER_SENSOR) && EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_ACCELEROMETER

/* Private variables ------------------------------------------------------- */
static float acc_buf[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
static int acc_sample_count = 0;

static bool acc_data_callback(const void *sample_buf, uint32_t byteLength)
{
    float *buffer = (float *)sample_buf;
    for(int i = 0; i < (int)(byteLength / sizeof(float)); i++) {
        acc_buf[acc_sample_count + i] = buffer[i];
    }

    return true;
}

// static void acc_read_data(float *values, size_t value_size)
// {
//     for (size_t i = 0; i < value_size; i++) {
//         values[i] = acc_buf[i];
//     }
// }

void run_nn(bool debug)
{
    bool stop_inferencing = false;
    // summary of inferencing settings (from model_metadata.h)
    ei_printf("Inferencing settings:\n");
    ei_printf("\tInterval: ");
    ei_printf_float((float)EI_CLASSIFIER_INTERVAL_MS);
    ei_printf("ms.\n");
    ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    ei_printf("\tSample length: ");
    ei_printf_float(1000.0f * static_cast<float>(EI_CLASSIFIER_RAW_SAMPLE_COUNT) /
                  (1000.0f / static_cast<float>(EI_CLASSIFIER_INTERVAL_MS)));
    ei_printf("ms.\n");
    ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) /
                                            sizeof(ei_classifier_inferencing_categories[0]));

    ei_printf("Starting inferencing, press 'b' to break\n");

    if(ei_inertial_sample_start(&acc_data_callback, EI_CLASSIFIER_INTERVAL_MS) == false) {
        return;
    }

    while (stop_inferencing == false) {
        ei_printf("Starting inferencing in 2 seconds...\n");

        // instead of wait_ms, we'll wait on the signal, this allows threads to cancel us...
        if (ei_sleep(2000) != EI_IMPULSE_OK) {
            break;
        }

        if(ei_user_invoke_stop()) {
            ei_printf("Inferencing stopped by user\r\n");
            break;
        }

        if (stop_inferencing) {
            break;
        }

        ei_printf("Sampling...\n");

            /* Run sampler */
        acc_sample_count = 0;
        for(int i = 0; i < EI_CLASSIFIER_RAW_SAMPLE_COUNT; i++) {
            ei_inertial_read_data();
            acc_sample_count += EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME;
        }

        // Create a data structure to represent this window of data
        signal_t signal;
        int err = numpy::signal_from_buffer(acc_buf, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
        if (err != 0) {
            ei_printf("ERR: signal_from_buffer failed (%d)\n", err);
        }

        // run the impulse: DSP, neural network and the Anomaly algorithm
        ei_impulse_result_t result = { 0 };
        EI_IMPULSE_ERROR ei_error = run_classifier(&signal, &result, debug);
        if (ei_error != EI_IMPULSE_OK) {
            ei_printf("Failed to run impulse (%d)\n", ei_error);
            break;
        }

        ei_print_results(&ei_default_impulse, &result);

        /*BLE PRINTF*/
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            sprintf(ble_printf, "%s: %f\n", result.classification[ix].label, result.classification[ix].value);
            ble_nus_send_data(ble_printf, strlen(ble_printf));
        }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
        sprintf(ble_printf, "anomaly score: %f\n", result.anomaly);
        ble_nus_send_data(ble_printf, strlen(ble_printf));
#endif

        memset(ble_printf, 0x00, sizeof(ble_printf));
        /*END BLE PRINTF*/

        if(ei_user_invoke_stop() || ei_ble_user_invoke_stop()) {
            ei_printf("Inferencing stopped by user\r\n");
            ble_nus_send_data("Inferencing stopped by user\n", strlen("Inferencing stopped by user\n"));
            memset(ble_printf, 0x00, sizeof(ble_printf));
            break;
        }
    }
}
#elif defined(EI_CLASSIFIER_SENSOR) && EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_MICROPHONE
void run_nn(bool debug)
{
    if (EI_CLASSIFIER_FREQUENCY > 16000) {
        ei_printf("ERR: Frequency is %d but can not be higher then 16000Hz\n", (int)EI_CLASSIFIER_FREQUENCY);
        return;
    }

    extern signal_t ei_microphone_get_signal();

    // summary of inferencing settings (from model_metadata.h)
    ei_printf("Inferencing settings:\n");
    ei_printf("\tInterval: ");
    ei_printf_float((float)EI_CLASSIFIER_INTERVAL_MS);
    ei_printf("ms.\n");
    ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    ei_printf("\tSample length: %d ms.\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT / 16);
    ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) /
                                            sizeof(ei_classifier_inferencing_categories[0]));

    if (ei_microphone_inference_start(EI_CLASSIFIER_RAW_SAMPLE_COUNT, EI_CLASSIFIER_INTERVAL_MS) == false) {
        ei_printf("ERR: Could not allocate audio buffer (size %d), this could be due to the window length of your model\r\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT);
        return;
    }

    ei_printf("Starting inferencing, press 'b' to break\n");

    while (1) {
        ei_printf("Starting inferencing in 2 seconds...\n");

        // instead of wait_ms, we'll wait on the signal, this allows threads to cancel us...
        if (ei_sleep(2000) != EI_IMPULSE_OK) {
            break;
        }

        if(ei_user_invoke_stop()) {
            ei_printf("Inferencing stopped by user\r\n");
            break;
        }

        ei_printf("Recording...\n");

        ei_microphone_inference_reset_buffers();
        bool m = ei_microphone_inference_record();
        if (!m) {
            ei_printf("ERR: Failed to record audio...\n");
            break;
        }

        ei_printf("Recording done\n");

        signal_t signal;
        signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
        signal.get_data = &ei_microphone_audio_signal_get_data;
        ei_impulse_result_t result = {0};

        EI_IMPULSE_ERROR r = run_classifier(&signal, &result, debug);
        if (r != EI_IMPULSE_OK) {
            ei_printf("ERR: Failed to run classifier (%d)\n", r);
            break;
        }

        ei_print_results(&ei_default_impulse, &result);

        /*BLE PRINTF*/
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            sprintf(ble_printf, "%s: %f\n", result.classification[ix].label, result.classification[ix].value);
            ble_nus_send_data(ble_printf, strlen(ble_printf));
        }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
        sprintf(ble_printf, "anomaly score: %f\n", result.anomaly);
        ble_nus_send_data(ble_printf, strlen(ble_printf));
#endif

        memset(ble_printf, 0x00, sizeof(ble_printf));
        /*END BLE PRINTF*/

        if(ei_user_invoke_stop() || ei_ble_user_invoke_stop()) {
            ei_printf("Inferencing stopped by user\r\n");
            ble_nus_send_data("Inferencing stopped by user\n", strlen("Inferencing stopped by user\n"));
            memset(ble_printf, 0x00, sizeof(ble_printf));
            break;
        }
    }

    ei_microphone_inference_end();
}

void run_nn_continuous(bool debug)
{
    bool stop_inferencing = false;
    if (EI_CLASSIFIER_FREQUENCY > 16000) {
        ei_printf("ERR: Frequency is %d but can not be higher then 16000Hz\n", (int)EI_CLASSIFIER_FREQUENCY);
        return;
    }

    int print_results = -(EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW);
    // summary of inferencing settings (from model_metadata.h)
    ei_printf("Inferencing settings:\n");
    ei_printf("\tInterval: ");
    ei_printf_float((float)EI_CLASSIFIER_INTERVAL_MS);
    ei_printf("ms.\n");
    ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    ei_printf("\tSample length: %d ms.\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT / 16);
    ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) /
                                            sizeof(ei_classifier_inferencing_categories[0]));

    ei_printf("Starting inferencing, press 'b' to break\n");

    run_classifier_init();
    if(ei_microphone_inference_start(EI_CLASSIFIER_SLICE_SIZE, EI_CLASSIFIER_INTERVAL_MS) == false) {
        ei_printf("ERR: Failed to start microphone\r\n");
    }

    while (stop_inferencing == false) {

        bool m = ei_microphone_inference_record();
        if (!m) {
            ei_printf("ERR: Failed to record audio...\n");
            break;
        }

        signal_t signal;
        signal.total_length = EI_CLASSIFIER_SLICE_SIZE;
        signal.get_data = &ei_microphone_audio_signal_get_data;
        ei_impulse_result_t result = {0};

        EI_IMPULSE_ERROR r = run_classifier_continuous(&signal, &result, debug);
        if (r != EI_IMPULSE_OK) {
            ei_printf("ERR: Failed to run classifier (%d)\n", r);
            break;
        }

        if (++print_results >= (EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW >> 1)) {
            ei_print_results(&ei_default_impulse, &result);
            print_results = 0;
        }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
        sprintf(ble_printf, "anomaly score: %f\n", result.anomaly);
        ble_nus_send_data(ble_printf, strlen(ble_printf));
#endif

        memset(ble_printf, 0x00, sizeof(ble_printf));
        /*END BLE PRINTF*/

        if(ei_user_invoke_stop() || ei_ble_user_invoke_stop()) {
            ei_printf("Inferencing stopped by user\r\n");
            ble_nus_send_data("Inferencing stopped by user\n", strlen("Inferencing stopped by user\n"));
            memset(ble_printf, 0x00, sizeof(ble_printf));
            break;
        }
    }

    ei_microphone_inference_end();
    run_classifier_deinit();
}

#else
void run_nn(bool debug) {}
#error "EI_CLASSIFIER_SENSOR not configured, cannot configure `run_nn`"

#endif  // EI_CLASSIFIER_SENSOR

void run_nn_continuous_normal()
{
#if defined(EI_CLASSIFIER_SENSOR) && EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_MICROPHONE
    run_nn_continuous(false);
#else
    ei_printf("Error no continuous classification available for current model\r\n");
#endif
}

void run_nn_normal(void)
{
    run_nn(false);
}
