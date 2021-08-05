#include <device/sensirion.h>
#include <driver/i2c.h>
#include <esp_log.h>
#include <libesp.h>
#include <math.h>

#include "device/sfa30.h"

static const char* TAG = "sfa30";

struct sfa30_cmd_exec {
    sensirion_cmd_def_t def;
};

struct sfa30_cmd_read {
    sensirion_cmd_def_t def;
};

const sfa30_cmd_exec_t SFA30_CMD_START_CONTINUOUS_MEASUREMENT = {
    .def = {.code = 0x0006, .delay_ms = 20}};
const sfa30_cmd_exec_t SFA30_CMD_STOP_CONTINUOUS_MEASUREMENT = {
    .def = {.code = 0x0104, .delay_ms = 20}};

const sfa30_cmd_read_t SFA30_CMD_READ_MEASURED_VALUES = {
    .def = {.code = 0x0327, .delay_ms = 5}};
const sfa30_cmd_read_t SFA30_CMD_GET_DEVICE_MARKING = {
    .def = {.code = 0xD060, .delay_ms = 5}};

// Unfortunately, delay not specified in datasheet.
const sfa30_cmd_exec_t SFA30_CMD_RESET = {
    .def = {.code = 0xD304, .delay_ms = 100}};

#define POLLING_INTERVAL_MS 250
#define POLLING_MAX_ITERS (3 * 4)

// Poll the device `dev` for its product type, retrying some number of times if
// we get a failure, in order to allow the device to start up. (Unfortunately,
// the spec does not give a maximum startup time.)
static esp_err_t polling_read_device_marking(sfa30_handle_t dev,
                                             uint8_t device_marking[33]) {
    esp_err_t ret;

    for (size_t i = 0; i < POLLING_MAX_ITERS; i++) {
        ret = sfa30_cmd_read(dev, &SFA30_CMD_GET_DEVICE_MARKING,
                             (uint16_t*) device_marking, 16);

        if (ret == ESP_OK) {
            device_marking[32] = '\0';
            for (size_t j = 0; j < 16; j++) {
                uint8_t tmp = device_marking[2 * j];
                device_marking[2 * j] = device_marking[(2 * j) + 1];
                device_marking[(2 * j) + 1] = tmp;
            }

            return ESP_OK;
        }
    }

    return ret;
}

esp_err_t sfa30_init(i2c_port_t port, uint8_t addr, sfa30_handle_t* out_dev) {
    esp_err_t ret;

    sfa30_handle_t dev;
    sensirion_init(port, addr, &dev);

    // Try to contact the device, retrying (up to a maximum number of times) if
    // we get a failure, in order to allow the device time to start up.
    uint8_t serial[33];
    ret = polling_read_device_marking(dev, serial);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG,
                 "I2C read failed (0x%X), are I2C pin numbers/address correct?",
                 ret);
        goto sfa30_init_fail;
    }

    ESP_LOGD(TAG, "serial=<str>: %s", (char*) serial);
    ESP_LOGD(TAG, "   hex= [00]: %02X %02X %02X %02X %02X %02X %02X %02X",
             serial[0x00], serial[0x01], serial[0x02], serial[0x03],
             serial[0x04], serial[0x05], serial[0x06], serial[0x07]);
    ESP_LOGD(TAG, "        [08]: %02X %02X %02X %02X %02X %02X %02X %02X",
             serial[0x08], serial[0x09], serial[0x0A], serial[0x0B],
             serial[0x0C], serial[0x0D], serial[0x0E], serial[0x0F]);
    ESP_LOGD(TAG, "        [10]: %02X %02X %02X %02X %02X %02X %02X %02X",
             serial[0x10], serial[0x11], serial[0x12], serial[0x13],
             serial[0x14], serial[0x15], serial[0x16], serial[0x17]);
    ESP_LOGD(TAG, "        [18]: %02X %02X %02X %02X %02X %02X %02X %02X",
             serial[0x18], serial[0x19], serial[0x1A], serial[0x1B],
             serial[0x1C], serial[0x1D], serial[0x1E], serial[0x1F]);

    ret = sfa30_reset(dev);
    if (ret != ESP_OK) {
        goto sfa30_init_fail;
    }

    *out_dev = dev;
    return ESP_OK;

sfa30_init_fail:
    sfa30_destroy(dev);
    return ret;
}

void sfa30_destroy(sfa30_handle_t dev) {
    ESP_ERROR_DISCARD(sfa30_cmd_exec(dev, &SFA30_CMD_RESET));
    sensirion_destroy(dev);
}

esp_err_t sfa30_reset(sfa30_handle_t dev) {
    esp_err_t ret;

    // Reset the device.
    ret = sfa30_cmd_exec(dev, &SFA30_CMD_RESET);
    if (ret != ESP_OK) {
        return ret;
    }

    // Try to contact the device, retrying (up to a maximum number of times) if
    // we get a failure, in order to allow the device time to start up. (Spec
    // does not give a maximum startup time.)
    uint8_t device_marking[33];
    return polling_read_device_marking(dev, device_marking);
}

esp_err_t sfa30_cmd_exec(sfa30_handle_t dev, const sfa30_cmd_exec_t* cmd) {
    return sensirion_cmd_perform(dev, &cmd->def, NULL, 0, NULL, 0);
}

esp_err_t sfa30_cmd_read(sfa30_handle_t dev, const sfa30_cmd_read_t* cmd,
                         uint16_t* in_data, size_t in_count) {
    return sensirion_cmd_perform(dev, &cmd->def, NULL, 0, in_data, in_count);
}

void sfa30_decode_h2co(int16_t raw_h2co, double* h2co_ppb) {
    *h2co_ppb = ((double) raw_h2co) / 5.0;
}

void sfa30_decode_temp(int16_t raw_temp, double* temp_c) {
    *temp_c = ((double) raw_temp) / 200.0;
}

void sfa30_decode_hum(int16_t raw_hum, double* rel_humidity) {
    *rel_humidity = ((double) raw_hum) / 100.0;
}
