/* Edge Impulse ingestion SDK
 * Copyright (c) 2021 EdgeImpulse Inc.
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

/* Include ----------------------------------------------------------------- */
#include "ei_microphone.h"
#include "ei_zephyr_flash_commands.h"
#include "ei_device_nordic_nrf52.h"
#include "ei_classifier_porting.h"

#include "ei_config_types.h"
#include "sensor_aq_mbedtls_hs256.h"
#include "sensor_aq_none.h"

#include <nrfx_pdm.h>
#include <string.h>

/* Read nordic id from the registers */
#if ((CONFIG_SOC_NRF52840 == 1) || \
       (CONFIG_SOC_NRF52840_QIAA == 1))
#define PDM_CLK_PIN                         36// 32+4 = p1.04
#define PDM_DIN_PIN                         37// 32+5 = p1.05
#elif ((CONFIG_SOC_NRF5340_CPUAPP == 1) || \
       (CONFIG_SOC_NRF5340_CPUAPP_QKAA == 1))
#define PDM_CLK_PIN                         37// 32+5 = p1.05
#define PDM_DIN_PIN                         38// 32+6 = p1.06
#else 
#error "Unsupported build target was chosen!"
#endif

/* Audio sampling config */
#define AUDIO_SAMPLING_FREQUENCY            16000
#define AUDIO_SAMPLES_PER_MS                (AUDIO_SAMPLING_FREQUENCY / 1000)
#define AUDIO_DSP_SAMPLE_LENGTH_MS          16
#define AUDIO_DSP_SAMPLE_RESOLUTION         (sizeof(short))
#define AUDIO_DSP_SAMPLE_BUFFER_SIZE        (AUDIO_SAMPLES_PER_MS * AUDIO_DSP_SAMPLE_LENGTH_MS * AUDIO_DSP_SAMPLE_RESOLUTION) //4096

/* Buffers for receiving PDM mic data */
static int16_t pdm_buffer_temp[2][AUDIO_DSP_SAMPLE_BUFFER_SIZE] = {0};
int16_t *current_buff;
bool write_data = false;

/** Status and control struct for inferencing struct */
typedef struct {
    int16_t *buffers[2];
    uint8_t buf_select;
    uint8_t buf_ready;
    uint32_t buf_count;
    uint32_t n_samples;
} inference_t;

extern ei_config_t *ei_config_get_config();

/* Dummy functions for sensor_aq_ctx type */
static size_t ei_write(const void*, size_t size, size_t count, EI_SENSOR_AQ_STREAM*)
{
    return count;
}

static int ei_seek(EI_SENSOR_AQ_STREAM*, long int offset, int origin)
{
    return 0;
}

/* Private variables ------------------------------------------------------- */
static bool record_ready = false;
static uint32_t headerOffset;
static uint32_t samples_required;
static uint32_t current_sample;

static inference_t inference;
static int16_t max_audio_lvl = 0;

static unsigned char ei_mic_ctx_buffer[1024];
static sensor_aq_signing_ctx_t ei_mic_signing_ctx;
static sensor_aq_mbedtls_hs256_ctx_t ei_mic_hs_ctx;
static sensor_aq_ctx ei_mic_ctx = {
    { ei_mic_ctx_buffer, 1024 },
    &ei_mic_signing_ctx,
    &ei_write,
    &ei_seek,
    NULL,
};

/* Private functions ------------------------------------------------------- */

/**
 * @brief      Ingestion audio callback, write audio samples to memory
 *             Signal record_ready when all needed samples are there
 * @param      buffer   Pointer to source buffer
 * @param[in]  n_bytes  Number of bytes to write
 */
static void audio_buffer_callback(void *buffer, uint32_t n_bytes)
{

    if((current_sample + n_bytes) > (samples_required << 1)) {
        n_bytes = (samples_required << 1) - current_sample;
    }

    ei_zephyr_flash_write_samples((const void *)buffer, headerOffset + current_sample, n_bytes);

    ei_mic_ctx.signature_ctx->update(ei_mic_ctx.signature_ctx, (uint8_t*)buffer, n_bytes);

    current_sample += n_bytes;
    if(current_sample >= (samples_required << 1)) {
        record_ready = false;
    }
}

/**
 * @brief      Inference audio callback, store samples in ram buffer
 *             Signal when buffer is full, and swap buffers
 * @param      buffer   Pointer to source buffer
 * @param[in]  n_bytes  Number of bytes to write
 */
static void audio_buffer_inference_callback(void *buffer, uint32_t n_bytes)
{
    int16_t *samples = (int16_t *)buffer;

    for(uint32_t i = 0; i < (n_bytes >> 1); i++) {
        inference.buffers[inference.buf_select][inference.buf_count++] = samples[i];

        if(inference.buf_count >= inference.n_samples) {
            inference.buf_select ^= 1;
            inference.buf_count = 0;
            inference.buf_ready = 1;
        }
    }
}

/**
 * @brief Get the max diff value from the sample buffer
 * @param buffer Pointer to source buffer
 * @param n_bytes Number of bytes in buffer
 */
static void audio_sanity_check_callback(void *buffer, uint32_t n_bytes)
{
    int16_t *samples = (int16_t *)buffer;
    int16_t prev_sample = samples[0];

    for(uint32_t i = 1; i < (n_bytes >> 1); i++) {

        int16_t diff_sample = abs(prev_sample - samples[i]);
        if(max_audio_lvl < diff_sample) {
            max_audio_lvl = diff_sample;
        }
        prev_sample = samples[i];
    }
    record_ready = false;
}

/**
 * @brief      PDM receive data handler
 * @param[in]  p_evt  pdm event structure
 */
static void pdm_data_handler(nrfx_pdm_evt_t const * p_evt)
{
    nrfx_err_t err = NRFX_SUCCESS;
    static uint8_t buf_toggle = 0;

    if(p_evt->error != 0){
        ei_printf("PDM handler error ocured\n");
        ei_printf("pdm_data_handler error: %d, %d  \n", p_evt->error, p_evt->buffer_requested);
        return;
    }
    if(true == p_evt->buffer_requested){
        buf_toggle ^= 1;
        err = nrfx_pdm_buffer_set(pdm_buffer_temp[buf_toggle], AUDIO_DSP_SAMPLE_BUFFER_SIZE);
        if(err != NRFX_SUCCESS){
            ei_printf("PDM buffer init error: %d\n", err);
        }
    }
    if(p_evt->buffer_released != NULL){
            write_data = true;
            current_buff = pdm_buffer_temp[buf_toggle];
    }
}

/**
 * @brief      PDM receive data handler
 * @param[in]  p_evt  pdm event structure
 */
static void pdm_inference_data_handler(nrfx_pdm_evt_t const * p_evt)
{
    nrfx_err_t err = NRFX_SUCCESS;
    static uint8_t buf_toggle = 0;

    if(p_evt->error != 0){
        ei_printf("PDM handler error ocured\n");
        ei_printf("pdm_data_handler error: %d, %d  \n", p_evt->error, p_evt->buffer_requested);
        return;
    }
    if(true == p_evt->buffer_requested){
        buf_toggle ^= 1;
        err = nrfx_pdm_buffer_set(pdm_buffer_temp[buf_toggle], AUDIO_DSP_SAMPLE_BUFFER_SIZE);
        if(err != NRFX_SUCCESS){
            ei_printf("PDM buffer init error: %d\n", err);
        }
    }
    if(p_evt->buffer_released != NULL){
        current_buff = pdm_buffer_temp[buf_toggle];
        audio_buffer_inference_callback(&pdm_buffer_temp[buf_toggle], sizeof(pdm_buffer_temp)/2);
    }
}

/**
 * @brief      Capture 2 channel pdm data every 100 ms.
 *             Waits for new data to be ready.
 *             Creates a 1 channel pdm array and calls callback function
 * @param[in]  callback  Callback needs to handle the audio samples
 */
static void get_dsp_data(void (*callback)(void *buffer, uint32_t n_bytes))
{
    //TODO: check the number of bytes
    if(write_data == true){
        callback((void *)current_buff, sizeof(pdm_buffer_temp)/2);
        write_data = false;
    }
}

static void finish_and_upload(char *filename, uint32_t sample_length_ms) {

    ei_printf("Done sampling, total bytes collected: %u\n", (samples_required << 1));


    ei_printf("[1/1] Uploading file to Edge Impulse...\n");

    ei_printf("Not uploading file, not connected to WiFi. Used buffer, from=%lu, to=%lu.\n", 0, (samples_required << 1) + headerOffset);


    ei_printf("[1/1] Uploading file to Edge Impulse OK (took %d ms.)\n", 200);//upload_timer.read_ms());

    ei_printf("OK\n");

    EiDevice.set_state(eiStateIdle);
}

static int insert_ref(char *buffer, int hdrLength)
{
    #define EXTRA_BYTES(a)  ((a & 0x3) ? 4 - (a & 0x3) : (a & 0x03))
    const char *ref = "Ref-BINARY-i16";
    int addLength = 0;
    int padding = EXTRA_BYTES(hdrLength);

    buffer[addLength++] = 0x60 + 14 + padding;
    for(size_t i = 0; i < strlen(ref); i++) {
        buffer[addLength++] = *(ref + i);
    }
    for(int i = 0; i < padding; i++) {
        buffer[addLength++] = ' ';
    }

    buffer[addLength++] = 0xFF;

    return addLength;
}

static bool create_header(void)
{
    sensor_aq_init_mbedtls_hs256_context(&ei_mic_signing_ctx, &ei_mic_hs_ctx, ei_config_get_config()->sample_hmac_key);

    sensor_aq_payload_info payload = {
        EiDevice.get_id_pointer(),
        EiDevice.get_type_pointer(),
        1000.0f / static_cast<float>(AUDIO_SAMPLING_FREQUENCY),
        { { "audio", "wav" } }
    };

    int tr = sensor_aq_init(&ei_mic_ctx, &payload, NULL, true);

    if (tr != AQ_OK) {
        ei_printf("ERR: sensor_aq_init failed (%d)\n", tr);
        return false;
    }

    // then we're gonna find the last byte that is not 0x00 in the CBOR buffer.
    // That should give us the whole header
    size_t end_of_header_ix = 0;
    for (size_t ix = ei_mic_ctx.cbor_buffer.len - 1; ix >= 0; ix--) {
        if (((uint8_t*)ei_mic_ctx.cbor_buffer.ptr)[ix] != 0x0) {
            end_of_header_ix = ix;
            break;
        }
    }

    if (end_of_header_ix == 0) {
        ei_printf("Failed to find end of header\n");
        return false;
    }


    int ref_size = insert_ref(((char*)ei_mic_ctx.cbor_buffer.ptr + end_of_header_ix), end_of_header_ix);

    // and update the signature
    tr = ei_mic_ctx.signature_ctx->update(ei_mic_ctx.signature_ctx, ((uint8_t*)ei_mic_ctx.cbor_buffer.ptr + end_of_header_ix), ref_size);
    if (tr != 0) {
        ei_printf("Failed to update signature from header (%d)\n", tr);
        return false;
    }

    end_of_header_ix += ref_size;

    // Write to blockdevice
    tr = ei_zephyr_flash_write_samples(ei_mic_ctx.cbor_buffer.ptr, 0, end_of_header_ix);

    if (tr != 0) {
        ei_printf("Failed to write to header blockdevice (%d)\n", tr);
        return false;
    }

    headerOffset = end_of_header_ix;

    return true;
}

/**
 * @brief Get a full sample buffer and run sanity check
 * @return true microphone is working
 * @return false stops audio and return
 */
static bool do_sanity_check(void)
{
    max_audio_lvl = 0;
    record_ready = true;
    write_data = false;

    while (record_ready == true) {
        get_dsp_data(&audio_sanity_check_callback);
    }

    if(max_audio_lvl < 10) {
        ei_printf("\r\nERR: No audio recorded, is the microphone connected?\r\n");
        nrfx_pdm_stop();
        EiDevice.set_state(eiStateFinished);
        return false;
    }
    else {
        return true;
    }
}

/**
 * @brief Set the up nrf pdm object, call pdm init
 *
 * @param event_handler
 * @return false on error
 */
static bool setup_nrf_pdm(nrfx_pdm_event_handler_t  event_handler)
{
    nrfx_err_t err;

    /* PDM driver configuration */
    nrfx_pdm_config_t config_pdm = NRFX_PDM_DEFAULT_CONFIG(PDM_CLK_PIN, PDM_DIN_PIN);
    config_pdm.clock_freq = NRF_PDM_FREQ_1280K;
    config_pdm.ratio = NRF_PDM_RATIO_80X;
    config_pdm.edge = NRF_PDM_EDGE_LEFTRISING;
    config_pdm.gain_l = NRF_PDM_GAIN_MAXIMUM;
    config_pdm.gain_r = NRF_PDM_GAIN_MAXIMUM;

    /* PDM interrupt configuration necessary for Zephyr */
#if ((CONFIG_SOC_NRF52840 == 1) || \
       (CONFIG_SOC_NRF52840_QIAA == 1))
    IRQ_DIRECT_CONNECT(PDM_IRQn, 6, nrfx_pdm_irq_handler, 0);
#elif ((CONFIG_SOC_NRF5340_CPUAPP == 1) || \
       (CONFIG_SOC_NRF5340_CPUAPP_QKAA == 1))
    IRQ_DIRECT_CONNECT(PDM0_IRQn, 6, nrfx_pdm_irq_handler, 0);
#else 
#error "Unsupported build target was chosen!"
#endif

    err = nrfx_pdm_init(&config_pdm, event_handler);
    if(err != NRFX_SUCCESS){
        return false;
    }
    else{
        return true;
    }
}

/* Public functions -------------------------------------------------------- */

/**
 * @brief      Set the PDM mic to +34dB
 * @return     false on error
 */
bool ei_microphone_init(void)
{
    return setup_nrf_pdm(pdm_data_handler);
}

bool ei_microphone_record(uint32_t sample_length_ms, uint32_t start_delay_ms, bool print_start_messages)
{
    nrfx_err_t err;
    
    EiDevice.set_state(eiStateErasingFlash);

    if (print_start_messages) {
        ei_printf("Starting in %lu ms... (or until all flash was erased)\n",
                  start_delay_ms);
    }

    nrfx_pdm_uninit();
    if(!setup_nrf_pdm(pdm_data_handler)) {
        return false;
    }
    err = nrfx_pdm_start();
    if(err != NRFX_SUCCESS){
        return false;
    }

    /* delay for the mic to erase the flash and overcome initial sound burst */
    if (start_delay_ms < 2000) {
        start_delay_ms = 2000;
        EiDevice.delay_ms(start_delay_ms);
    }
    /* Erase necessary flash size for new data */
    if (ei_zephyr_flash_erase_sampledata(0, (samples_required << 1) +
        ei_zephyr_flash_get_block_size()) != ZEPHYR_FLASH_CMD_OK) {
        nrfx_pdm_stop();
        return false;
    }

    /* Since we have no feedback from the PDM sensor, do a sanity check on the data stream */
    if(do_sanity_check() == false) {
        return false;
    }

    create_header();

    if (print_start_messages) {
        ei_printf("Sampling...\n");
    }

    return true;
}

bool ei_microphone_inference_start(uint32_t n_samples)
{
    nrfx_err_t err;

    nrfx_pdm_uninit();
    if(!setup_nrf_pdm(pdm_data_handler)) {
        return false;
    }
    err = nrfx_pdm_start();
    if(err != NRFX_SUCCESS){
        return false;
    }

    EiDevice.delay_ms(1000);
    /* Since we have no feedback from the PDM sensor, do a sanity check on the data stream */
    if(do_sanity_check() == false) {
        return false;
    }
    nrfx_pdm_stop();

    inference.buffers[0] = (int16_t *)malloc(n_samples * sizeof(int16_t));

    if(inference.buffers[0] == NULL) {
        return false;
    }

    inference.buffers[1] = (int16_t *)malloc(n_samples * sizeof(int16_t));

    if(inference.buffers[1] == NULL) {
        free(inference.buffers[0]);
        return false;
    }

    inference.buf_select = 0;
    inference.buf_count  = 0;
    inference.n_samples  = n_samples;
    inference.buf_ready  = 0;

    nrfx_pdm_uninit();
    if(!setup_nrf_pdm(pdm_inference_data_handler)) {
        return false;
    }
    err = nrfx_pdm_start();
    if(err != NRFX_SUCCESS){
        return false;
    }

    return true;
}

bool ei_microphone_inference_record(void)
{
    bool ret = true;

    if (inference.buf_ready == 1) {
        ei_printf(
            "Error sample buffer overrun. Decrease the number of slices per model window "
            "(EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW)\n");
        ret = false;
    }

    while (inference.buf_ready == 0) {
    };
 
    inference.buf_ready = 0;

    return ret;
}

/**
 * @brief      Reset buffer counters for non-continuous inferecing
 */
void ei_microphone_inference_reset_buffers(void)
{
    inference.buf_ready = 0;
    inference.buf_count = 0;
}

/**
 * Get raw audio signal data
 */
int ei_microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr)
{
    size_t i;

    for(i = 0; i < length; i++) {
        *(out_ptr + i) = (float)inference.buffers[inference.buf_select ^ 1][offset + i]
        / ((float)(1 << 15));
    }

    return 0;
}


bool ei_microphone_inference_end(void)
{
    uint32_t nrfx_err;
    record_ready = false;

    nrfx_err = nrfx_pdm_stop();
    if(nrfx_err != NRFX_SUCCESS)
    {
        ei_printf("PDM Could not start PDM sampling, error = %d", nrfx_err);
    }

    free(inference.buffers[0]);
    free(inference.buffers[1]);
    return true;
}

/**
 * Sample raw data
 */
bool ei_microphone_sample_start(void)
{
    uint32_t nrfx_err;

    ei_printf("Sampling settings:\n");
    ei_printf("\tInterval: "); (ei_printf_float((float)ei_config_get_config()->sample_interval_ms));ei_printf(" ms.\n");
    ei_printf("\tLength: %lu ms.\n", ei_config_get_config()->sample_length_ms);
    ei_printf("\tName: %s\n", ei_config_get_config()->sample_label);
    ei_printf("\tHMAC Key: %s\n", ei_config_get_config()->sample_hmac_key);
    char filename[256];
    int fn_r = snprintf(filename, 256, "/fs/%s", ei_config_get_config()->sample_label);
    if (fn_r <= 0) {
        ei_printf("ERR: Failed to allocate file name\n");
        return false;
    }
    ei_printf("\tFile name: %s\n", filename);


    samples_required = (uint32_t)(((float)ei_config_get_config()->sample_length_ms) / ei_config_get_config()->sample_interval_ms);

    /* Round to even number of samples for word align flash write */
    if(samples_required & 1) {
        samples_required++;
    }

    current_sample = 0;

    bool r = ei_microphone_record(ei_config_get_config()->sample_length_ms, (((samples_required <<1)/ ei_zephyr_flash_get_block_size()) * ZEPHYR_FLASH_BLOCK_ERASE_TIME_MS), true);
    if (!r) {
        return r;
    }
    record_ready = true;
    write_data = false;
    EiDevice.set_state(eiStateSampling);
    while(record_ready == true) {
        get_dsp_data(audio_buffer_callback);
    };

    nrfx_err = nrfx_pdm_stop();
    if(nrfx_err != NRFX_SUCCESS)
    {
        ei_printf("PDM Could not start PDM sampling, error = %d", nrfx_err);
    }

    int ctx_err = ei_mic_ctx.signature_ctx->finish(ei_mic_ctx.signature_ctx, ei_mic_ctx.hash_buffer.buffer);
    if (ctx_err != 0) {
        ei_printf("Failed to finish signature (%d)\n", ctx_err);
        return false;
    }

    // load the first page in flash...
    uint8_t *page_buffer = (uint8_t*)malloc(ei_zephyr_flash_get_block_size());
    if (!page_buffer) {
        ei_printf("Failed to allocate a page buffer to write the hash\n");
        return false;
    }

    int j = ei_zephyr_flash_read_samples(page_buffer, 0, ei_zephyr_flash_get_block_size());
    if (j != 0) {
        ei_printf("Failed to read first page (%d)\n", j);
        free(page_buffer);
        return false;
    }

    // update the hash
    uint8_t *hash = ei_mic_ctx.hash_buffer.buffer;
    // we have allocated twice as much for this data (because we also want to be able to represent in hex)
    // thus only loop over the first half of the bytes as the signature_ctx has written to those
    for (size_t hash_ix = 0; hash_ix < ei_mic_ctx.hash_buffer.size / 2; hash_ix++) {
        // this might seem convoluted, but snprintf() with %02x is not always supported e.g. by newlib-nano
        // we encode as hex... first ASCII char encodes top 4 bytes
        uint8_t first = (hash[hash_ix] >> 4) & 0xf;
        // second encodes lower 4 bytes
        uint8_t second = hash[hash_ix] & 0xf;

        // if 0..9 -> '0' (48) + value, if >10, then use 'a' (97) - 10 + value
        char first_c = first >= 10 ? 87 + first : 48 + first;
        char second_c = second >= 10 ? 87 + second : 48 + second;

        page_buffer[ei_mic_ctx.signature_index + (hash_ix * 2) + 0] = first_c;
        page_buffer[ei_mic_ctx.signature_index + (hash_ix * 2) + 1] = second_c;
    }

    j = ei_zephyr_flash_erase_sampledata(0, ei_zephyr_flash_get_block_size());
    if (j != 0) {
        ei_printf("Failed to erase first page (%d)\n", j);
        free(page_buffer);
        return false;
    }

    j = ei_zephyr_flash_write_samples(page_buffer, 0, ei_zephyr_flash_get_block_size());

    free(page_buffer);

    if (j != 0) {
        ei_printf("Failed to write first page with updated hash (%d)\n", j);
        return false;
    }


    finish_and_upload(filename, ei_config_get_config()->sample_length_ms);

    return true;
}