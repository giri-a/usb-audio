// i2s_functions.h
#ifndef _I2S_FUNCTIONS_H_
#define _I2S_FUNCTIONS_H_

#include "esp_idf_version.h"
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#include "driver/i2s_std.h"
#else
#include "driver/i2s.h"
#endif
#include "driver/gpio.h"


esp_err_t bsp_i2s_init(i2s_port_t i2s_num, uint32_t sample_rate);
esp_err_t bsp_i2s_reconfig(uint32_t sample_rate);
size_t bsp_i2s_read(void *data_buf, size_t count);
void bsp_i2s_write(void *data_buf, size_t count);
void decode_and_cancel_offset(int32_t *left_sample_p, int32_t *right_sample_p, bool reset);
void i2s_read_write_task();
extern uint16_t (*i2s_get_data)(void *data_buf, uint16_t count);
void i2s_consumer_func();

#endif
