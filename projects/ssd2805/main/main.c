#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event_loop.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "cam.h"
#include "ov2640.h"
#include "ov3660.h"
#include "sensor.h"
#include "sccb.h"
#include "lcd.h"
#include "jpeg.h"

static const char *TAG = "main";

#define  LCD_WR  GPIO_NUM_34
#define  LCD_RS  GPIO_NUM_1
#define  LCD_RD  GPIO_NUM_2
#define  LCD_CS  GPIO_NUM_21
#define  LCD_RST GPIO_NUM_18

#define  LCD_D0  GPIO_NUM_35
#define  LCD_D1  GPIO_NUM_37
#define  LCD_D2  GPIO_NUM_36
#define  LCD_D3  GPIO_NUM_39
#define  LCD_D4  GPIO_NUM_38
#define  LCD_D5  GPIO_NUM_41
#define  LCD_D6  GPIO_NUM_40
#define  LCD_D7  GPIO_NUM_45

// #define  LCD_D8   GPIO_NUM_21
// #define  LCD_D9   GPIO_NUM_18
// #define  LCD_D10  GPIO_NUM_17
// #define  LCD_D11  GPIO_NUM_16
// #define  LCD_D12  GPIO_NUM_15
// #define  LCD_D13  GPIO_NUM_14
// #define  LCD_D14  GPIO_NUM_13
// #define  LCD_D15  GPIO_NUM_12

void app_main() 
{
    lcd_config_t lcd_config = {
        .bit_width = 8,
        .clk_fre = 1 * 1000 * 1000,
        .pin_clk = LCD_WR,
        .pin_dc = LCD_RS,
        .pin_rd = LCD_RD,
        .pin_cs = LCD_CS,
        .pin_rst = LCD_RST,
        .pin_bk = -1,
        .pin_data = {LCD_D0, LCD_D1, LCD_D2, LCD_D3, LCD_D4, LCD_D5, LCD_D6, LCD_D7},
        .horizontal = 2, // 2: UP, 3： DOWN
        .max_buffer_size = 8 * 1024,
        .dis_invert = false,
        .dis_bgr = false
    };

    lcd_init(&lcd_config);

    uint16_t *test_buf = (uint16_t *)heap_caps_malloc(sizeof(uint16_t) * 800 * 480, MALLOC_CAP_SPIRAM);

    for (int x = 0; x < 320 * 480; x++) {
        test_buf[x] = 0xFFFF;
    }

    while (1) {
        lcd_set_index(0, 0, 479, 319);
        lcd_write_data((uint8_t *)test_buf, 320 * 480 * 2);
    }

}