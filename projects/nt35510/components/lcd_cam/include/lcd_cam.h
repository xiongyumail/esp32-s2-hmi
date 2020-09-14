#pragma once

#include "driver/gpio.h"
#include "driver/i2s.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LCD_DATA_WIDTH (16)
#define CAM_DATA_WIDTH (16)

typedef struct {
    bool en;
    uint8_t  width;
    uint32_t fre;
    struct {
        int8_t clk;
        int8_t data[LCD_DATA_WIDTH];
    } pin;
    struct {
        bool clk;
        bool data[LCD_DATA_WIDTH];
    } invert;
    uint32_t max_buffer_size; // DMA used
} lcd_config_t;

typedef struct {
    bool en;
    uint8_t  width;
    uint32_t fre;
    struct {
        int8_t xclk;
        int8_t pclk;
        int8_t vsync;
        int8_t href;
        int8_t data[CAM_DATA_WIDTH];
    } pin;
    struct {
        bool xclk;
        bool pclk;
        bool vsync;
        bool href;
        bool data[CAM_DATA_WIDTH];
    } invert;
    union {
        struct {
            uint32_t jpeg:   1; 
        };
        uint32_t val;
    } mode;
    uint32_t max_dma_buffer_size; // DMA used
    uint32_t recv_size;
    uint32_t frame_cnt;
    uint32_t frame_caps;
    uint32_t task_stack;
    uint8_t  task_pri;
} cam_config_t;

typedef struct {
    lcd_config_t lcd;
    cam_config_t cam; 
} lcd_cam_config_t;

void lcd_write_data(uint8_t *data, size_t len);
void cam_start(void);
void cam_stop(void);
size_t cam_take(uint8_t **buffer_p);
void cam_give(uint8_t *buffer);
int lcd_cam_init(lcd_cam_config_t *config);

#ifdef __cplusplus
}
#endif