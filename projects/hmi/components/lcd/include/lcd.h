#pragma once

#include "driver/gpio.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t clk_fre;
    int8_t pin_clk;
    int8_t pin_mosi;
    int8_t pin_dc;
    int8_t pin_cs;
    int8_t pin_rst;
    int8_t pin_bk;
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