#pragma once

#include "driver/gpio.h"
#include "driver/i2s.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t bit_width;
    uint32_t clk_fre;
    int8_t pin_clk;
    int8_t pin_dc; // rs
    int8_t pin_rd;
    int8_t pin_cs;
    int8_t pin_rst;
    int8_t pin_bk;
    int8_t pin_data[16];
    uint8_t horizontal;
    uint32_t max_buffer_size; // DMA used
    uint8_t dis_invert;
    uint8_t dis_bgr;
} lcd_config_t;

void lcd_rst();

void lcd_write_data(uint8_t *data, size_t len);

void lcd_set_index(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end);

int lcd_init(lcd_config_t *config);

#ifdef __cplusplus
}
#endif