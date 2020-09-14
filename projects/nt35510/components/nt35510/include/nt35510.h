#pragma once

#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*nt35510_write_callback_t) (uint8_t *data, size_t len);

typedef struct {
    uint8_t  width;
    struct {
        int8_t dc;
        int8_t rd;
        int8_t cs;
        int8_t rst;
        int8_t bk;
    } pin;
    struct {
        bool dc;
        bool rd;
        bool cs;
        bool rst;
        bool bk;
    } invert;
    uint8_t horizontal;
    uint8_t dis_invert;
    uint8_t dis_bgr;
    nt35510_write_callback_t write_cb;
} nt35510_config_t;

void nt35510_write_data(uint8_t *data, size_t len);
void nt35510_set_index(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end);
void nt35510_deinit(void);
int nt35510_init(nt35510_config_t *config);

#ifdef __cplusplus
}
#endif