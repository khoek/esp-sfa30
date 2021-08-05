#pragma once

#include <device/sensirion.h>
#include <driver/i2c.h>

typedef struct sfa30_cmd_exec sfa30_cmd_exec_t;
typedef struct sfa30_cmd_read sfa30_cmd_read_t;

extern const sfa30_cmd_exec_t SFA30_CMD_START_CONTINUOUS_MEASUREMENT;
extern const sfa30_cmd_exec_t SFA30_CMD_STOP_CONTINUOUS_MEASUREMENT;

extern const sfa30_cmd_read_t SFA30_CMD_READ_MEASURED_VALUES;
extern const sfa30_cmd_read_t SFA30_CMD_GET_DEVICE_MARKING;

extern const sfa30_cmd_exec_t SFA30_CMD_RESET;

typedef sensirion_dev_handle_t sfa30_handle_t;

// Register the SFA30 on the given I2C bus.
__result_use_check esp_err_t sfa30_init(i2c_port_t port, uint8_t addr,
                                        sfa30_handle_t* out_dev);

// Release the given handle.
void sfa30_destroy(sfa30_handle_t dev);

// Reset the device.
__result_use_check esp_err_t sfa30_reset(sfa30_handle_t dev);

// Perform a command over I2C. Use of these functions is thread-safe.
__result_use_check esp_err_t sfa30_cmd_exec(sfa30_handle_t dev,
                                            const sfa30_cmd_exec_t* cmd);
__result_use_check esp_err_t sfa30_cmd_read(sfa30_handle_t dev,
                                            const sfa30_cmd_read_t* cmd,
                                            uint16_t* in_data, size_t in_count);

// Decode formaldehyde (H2CO) concentration values returned by the SFA30.
void sfa30_decode_h2co(int16_t raw_h2co, double* h2co_ppb);

// Decode temperature values returned by the SFA30.
void sfa30_decode_temp(int16_t raw_temp, double* temp_c);

// Decode humidity values returned by the SFA30.
void sfa30_decode_hum(int16_t raw_hum, double* rel_humidity);
