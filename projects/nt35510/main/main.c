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
#include "ov2640.h"
#include "ov3660.h"
#include "sensor.h"
#include "sccb.h"
#include "lcd_cam.h"
#include "nt35510.h"
#include "jpeg.h"

static const char *TAG = "main";

#define JPEG_MODE 1
#define YUV_MODE 0
#define JPEG_ENCODE 0

#define LCD_WIDTH   (800)
#define LCD_HIGH    (480)
#define CAM_WIDTH   (320)
#define CAM_HIGH    (240)

#define  LCD_WR   GPIO_NUM_4
#define  LCD_RS   GPIO_NUM_3
#define  LCD_RD   GPIO_NUM_2

#define  LCD_D0   GPIO_NUM_20
#define  LCD_D1   GPIO_NUM_19
#define  LCD_D2   GPIO_NUM_18
#define  LCD_D3   GPIO_NUM_17
#define  LCD_D4   GPIO_NUM_16
#define  LCD_D5   GPIO_NUM_15
#define  LCD_D6   GPIO_NUM_14
#define  LCD_D7   GPIO_NUM_13

#define  LCD_D8   GPIO_NUM_12
#define  LCD_D9   GPIO_NUM_11
#define  LCD_D10  GPIO_NUM_10
#define  LCD_D11  GPIO_NUM_9
#define  LCD_D12  GPIO_NUM_8
#define  LCD_D13  GPIO_NUM_7
#define  LCD_D14  GPIO_NUM_6
#define  LCD_D15  GPIO_NUM_5

#define  CAM_XCLK  GPIO_NUM_1
#define  CAM_PCLK  GPIO_NUM_0
#define  CAM_VSYNC GPIO_NUM_2
#define  CAM_HREF  GPIO_NUM_3

#define  CAM_D0    GPIO_NUM_46
#define  CAM_D1    GPIO_NUM_45
#define  CAM_D2    GPIO_NUM_41
#define  CAM_D3    GPIO_NUM_42
#define  CAM_D4    GPIO_NUM_39
#define  CAM_D5    GPIO_NUM_40
#define  CAM_D6    GPIO_NUM_21
#define  CAM_D7    GPIO_NUM_38

#define  CAM_SCL   GPIO_NUM_7
#define  CAM_SDA   GPIO_NUM_8

#define CAM_PWD   -1
#define CAM_RST   -1

#define ITERATION 128 //迭代次数
#define REAL_CONSTANT 0.285f //实部常量
#define IMG_CONSTANT 0.01f //虚部常量
 
static uint16_t color_table[ITERATION];
 
//产生颜色表
static void InitCLUT(uint16_t *clut)
{
    uint32_t i = 0x00;
    uint16_t red = 0, green = 0, blue = 0;
 
    for (i = 0; i < ITERATION; i++) {
        red = (i * 8 * 256 / ITERATION) % 256;
        green = (i * 6 * 256 / ITERATION) % 256;
        blue = (i * 4 * 256 / ITERATION) % 256;//将 RGB888,转换为 RGB565
        red = red >> 3;
        red = red << 11;
        green = green >> 2;
        green = green << 5;
        blue = blue >> 3;
        clut[i] = red + green + blue;
    }
}
 
//产生 Julia 分形图形
//size_x,size_y:屏幕 x,y 方向的尺寸
//offset_x,offset_y:屏幕 x,y 方向的偏移
//zoom:缩放因子
static void GenerateJulia_fpu(uint16_t size_x, uint16_t size_y, uint16_t offset_x, uint16_t offset_y, uint16_t zoom, uint16_t *out_buffer)
{
    uint8_t i;
    uint16_t x, y;
    float tmp1, tmp2;
    float num_real, num_img;
    float radius;
 
    for (y = 0; y < size_y; y++) {
        for (x = 0; x < size_x; x++) {
            num_real = y - offset_y;
            num_real = num_real / zoom;
            num_img = x - offset_x;
            num_img = num_img / zoom;
            i = 0;
 
            radius = 0;
 
            while ((i < ITERATION - 1) && (radius < 4)) {
                tmp1 = num_real * num_real;
                tmp2 = num_img * num_img;
                num_img = 2 * num_real * num_img + IMG_CONSTANT;
                num_real = tmp1 - tmp2 + REAL_CONSTANT;
                radius = tmp1 + tmp2;
                i++;
            }
 
            out_buffer[y * size_x + x] = color_table[i];
        }
    }
}

static void lcd_cam_task(void *arg)
{
    #define LCD_TEST 1
    lcd_cam_config_t lcd_cam_config = {
        .lcd = {
            .en = LCD_TEST,
            .width = 16,
            .fre = 20 * 1000 * 1000,
            .pin = {
                .clk  = LCD_WR,
                .data = {LCD_D0, LCD_D1, LCD_D2, LCD_D3, LCD_D4, LCD_D5, LCD_D6, LCD_D7, LCD_D8, LCD_D9, LCD_D10, LCD_D11, LCD_D12, LCD_D13, LCD_D14, LCD_D15},
            },
            .invert = {
                .clk  = false,
                .data = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false},
            },
            .max_buffer_size = 32 * 1024,
        },
        .cam = {
            .en = !LCD_TEST,
            .width = 8,
            .fre = 16 * 1000 * 1000,
            .pin = {
                .xclk  = CAM_XCLK,
                .pclk  = CAM_PCLK,
                .vsync = CAM_VSYNC,
                .href = CAM_HREF,
                .data = {CAM_D0, CAM_D1, CAM_D2, CAM_D3, CAM_D4, CAM_D5, CAM_D6, CAM_D7},
            },
            .invert = {
                .xclk  = false,
                .pclk  = false,
                .vsync = true,
                .href = false,
                .data = {false, false, false, false, false, false, false, false},
            },
            .mode.jpeg = JPEG_MODE,
            .recv_size = CAM_WIDTH * CAM_HIGH * 2,
            .max_dma_buffer_size = 16 * 1024,
            .frame_cnt = 2, 
            .frame_caps = MALLOC_CAP_SPIRAM,
            .task_stack = 1024,
            .task_pri = configMAX_PRIORITIES
        }
    };

    nt35510_config_t nt35510_config = {
        .width = 16,
        .pin = {
            .dc  = LCD_RS,
            .rd = LCD_RD,
            .cs = -1,
            .rst = -1,
            .bk = -1,
        },
        .invert = {
            .dc  = false,
            .rd = false,
            .cs = false,
            .rst = false,
            .bk = false,
        },
        .horizontal = 2, // 2: UP, 3： DOWN
        .dis_invert = false,
        .dis_bgr = false,
        .write_cb = lcd_write_data
    };

#if LCD_TEST
    lcd_cam_init(&lcd_cam_config);
    nt35510_init(&nt35510_config);

    uint8_t *test_buf = (uint8_t *)heap_caps_malloc(sizeof(uint16_t) * LCD_WIDTH * LCD_HIGH, MALLOC_CAP_SPIRAM);

    extern const uint8_t pic_map[];
    for (int y = 0; y < LCD_HIGH; y++) {
        for (int x = 0; x < LCD_WIDTH * 2; x++) {
            test_buf[y * (LCD_WIDTH * 2) + x] = pic_map[y * (800 * 2) + x];
        }  
    }
    nt35510_set_index(0, 0, LCD_WIDTH - 1, LCD_HIGH - 1);
    nt35510_write_data((uint8_t *)test_buf, LCD_WIDTH * LCD_HIGH * 2);
    
    InitCLUT(color_table);
    uint32_t ticks_now = 0, ticks_last = 0;
    struct timeval now;   
    while (1) {
        gettimeofday(&now, NULL);
        ticks_last = now.tv_sec * 1000 + now.tv_usec / 1000;
        
        GenerateJulia_fpu(LCD_WIDTH, LCD_HIGH, LCD_WIDTH/2, LCD_HIGH/2, rand() * 10000, test_buf);
        nt35510_set_index(0, 0, LCD_WIDTH - 1, LCD_HIGH - 1);
        nt35510_write_data((uint8_t *)test_buf, LCD_WIDTH * LCD_HIGH * 2);
        gettimeofday(&now, NULL);
        ticks_now = now.tv_sec * 1000 + now.tv_usec / 1000;
        printf("fps: %.2f\n", 1000.0 / (int)(ticks_now - ticks_last));
    }
#else
#if JPEG_ENCODE
    uint8_t *jpeg_buf = (uint8_t *)heap_caps_malloc(CAM_WIDTH * CAM_HIGH * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
#endif

    lcd_cam_init(&lcd_cam_config);

    if (CAM_PWD != -1 && CAM_RST != -1) {
        gpio_config_t io_conf;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = (1ULL << CAM_PWD) | (1ULL << CAM_RST);
        io_conf.pull_down_en = 0;
        io_conf.pull_up_en = 0;
        gpio_config(&io_conf);

        gpio_set_level(CAM_PWD, 0);
        gpio_set_level(CAM_RST, 1);
    }

    sensor_t sensor;
    SCCB_Init(CAM_SDA, CAM_SCL);
    sensor.slv_addr = SCCB_Probe();
    ESP_LOGI(TAG, "sensor_id: 0x%x\n", sensor.slv_addr);
    if (sensor.slv_addr == 0x30) { // OV2640
        if (OV2640_Init(0, 1) != 0) {
            goto fail;
        }
#if JPEG_MODE
    OV2640_JPEG_Mode();
#else
#if YUV_MODE
    OV2640_YUV_Mode();
#else
    OV2640_RGB565_Mode(false);	//RGB565模式
#endif
#endif
        
        OV2640_ImageSize_Set(800, 600);
        OV2640_ImageWin_Set(0, 0, 800, 600);
        OV2640_OutSize_Set(CAM_WIDTH, CAM_HIGH); 
    } else if (sensor.slv_addr == 0x3C) { // OV3660
        ov3660_init(&sensor);
        sensor.init_status(&sensor);
        if (sensor.reset(&sensor) != 0) {
            goto fail;
        }
#if JPEG_MODE
    sensor.set_pixformat(&sensor, PIXFORMAT_JPEG);
#else
#if YUV_MODE
    sensor.set_pixformat(&sensor, PIXFORMAT_YUV422);
#else
    sensor.set_pixformat(&sensor, PIXFORMAT_RGB565);
#endif
#endif
        // totalX 变小，帧率提高
        // totalY 变小，帧率提高vsync 变短
        sensor.set_res_raw(&sensor, 0, 0, 2079, 1547, 8, 2, 1920, 800, CAM_WIDTH, CAM_HIGH, true, true);
        sensor.set_vflip(&sensor, 1);
        sensor.set_hmirror(&sensor, 1);
        sensor.set_pll(&sensor, false, 15, 1, 0, false, 0, true, 5); // 39 fps
    } else {
        ESP_LOGE(TAG, "sensor is temporarily not supported\n");
        goto fail;
    }

    ESP_LOGI(TAG, "camera init done\n");
    vTaskDelay(1000 / portTICK_RATE_MS);
    cam_start();
    uint32_t ticks_now = 0, ticks_last = 0;
    struct timeval now;
    while (1) {
        gettimeofday(&now, NULL);
        ticks_last = now.tv_sec * 1000 + now.tv_usec / 1000;
        uint8_t *cam_buf = NULL;
        size_t recv_len = 0;
        recv_len = cam_take(&cam_buf);
#if JPEG_MODE
        printf("total_len: %d\n", recv_len);
        for (int x = 0; x < 10; x++) {
            ets_printf("%d ", cam_buf[x]);
        }
        ets_printf("\n");

        int w, h;
        uint8_t *img = jpeg_decode(cam_buf, &w, &h);
        if (img) {
            ESP_LOGI(TAG, "jpeg: w: %d, h: %d\n", w, h);
            // nt35510_set_index(0, 0, w - 1, h - 1);
            // nt35510_write_data(img, w * h * sizeof(uint16_t));
            free(img);
        }
#else
#if JPEG_ENCODE
        int jpeg_len = jpeg_encode(YUV_MODE ? ENCODE_YUV_MODE: ENCODE_RGB16_MODE, cam_buf, CAM_WIDTH, CAM_HIGH, jpeg_buf, CAM_WIDTH * CAM_HIGH * sizeof(uint16_t));

        printf("jpeg_len: %d\n", jpeg_len);
        for (int x = 0; x < 10; x++) {
            ets_printf("%d ", jpeg_buf[x]);
        }
        ets_printf("\n");

        int w, h;
        uint8_t *img = jpeg_decode(jpeg_buf, &w, &h);
        if (img) {
            ESP_LOGI(TAG, "jpeg: w: %d, h: %d\n", w, h);
            nt35510_set_index(0, 0, w - 1, h - 1);
            nt35510_write_data(img, w * h * sizeof(uint16_t));
            free(img);
        }
#else
        nt35510_set_index(0, 0, CAM_WIDTH - 1, CAM_HIGH - 1);
        nt35510_write_data(cam_buf, CAM_WIDTH * CAM_HIGH * 2);
#endif
#endif
        cam_give(cam_buf);   
        gettimeofday(&now, NULL);
        ticks_now = now.tv_sec * 1000 + now.tv_usec / 1000;
        printf("fps: %.2f\n", 1000.0 / (int)(ticks_now - ticks_last));
    }

fail:
    // cam_deinit();
    vTaskDelete(NULL);
#endif
}

void app_main() 
{
    xTaskCreate(lcd_cam_task, "lcd_cam_task", 4096, NULL, 5, NULL);
}