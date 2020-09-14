#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp32s2/rom/lldesc.h"
#include "soc/system_reg.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "lcd_cam.h"

static const char *TAG = "lcd_cam";

#define LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE     (4095)
#define LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE     (4095)

typedef enum {
    CAM_IN_SUC_EOF_EVENT = 0,
    CAM_VSYNC_EVENT
} cam_event_t;

typedef struct {
    uint8_t *frame_buffer;
    size_t len;
} frame_buffer_event_t;

typedef struct {
    uint32_t dma_buffer_size;
    uint32_t dma_half_buffer_size;
    uint32_t dma_node_buffer_size;
    uint32_t dma_node_cnt;
    uint32_t dma_half_node_cnt;
    lldesc_t *dma;
    uint8_t *dma_buffer;
    QueueHandle_t event_queue;
} lcd_obj_t;

typedef struct {
    uint32_t dma_buffer_size;
    uint32_t dma_half_buffer_size;
    uint32_t dma_node_buffer_size;
    uint32_t dma_node_cnt;
    uint32_t frame_copy_cnt;
    lldesc_t *dma;
    uint8_t *dma_buffer;
    uint8_t *frame_buffer;
    uint8_t *frame_en;
    uint32_t frame_cnt;
    uint32_t recv_size;
    uint8_t jpeg_mode;
    uint8_t vsync_pin;
    uint8_t vsync_invert;
    QueueHandle_t event_queue;
    QueueHandle_t frame_buffer_queue;
    TaskHandle_t task_handle;
    intr_handle_t cam_intr_handle;
    intr_handle_t dma_intr_handle;
} cam_obj_t;

typedef struct {
    lcd_obj_t lcd;
    cam_obj_t cam;
    intr_handle_t intr_handle;
} lcd_cam_obj_t;

static lcd_cam_obj_t *lcd_cam_obj = NULL;

static void IRAM_ATTR lcd_cam_isr(void *arg)
{
    cam_event_t cam_event = {0};
    BaseType_t HPTaskAwoken = pdFALSE;
    typeof(I2S0.int_st) status = I2S0.int_st;
    I2S0.int_clr.val = status.val;
    // ets_printf("intr: 0x%x\n", int_st);

    if (status.out_eof) {
        xQueueSendFromISR(lcd_cam_obj->lcd.event_queue, &status.val, &HPTaskAwoken);
    }

    if (status.in_suc_eof) {
        cam_event = CAM_IN_SUC_EOF_EVENT;
        xQueueSendFromISR(lcd_cam_obj->cam.event_queue, (void *)&cam_event, &HPTaskAwoken);
    }

    if (HPTaskAwoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

void lcd_write_data(uint8_t *data, size_t len)
{
    int event  = 0;
    int x = 0, cnt = 0, size = 0;
    int end_pos = 0;
    if (len <= 0) {
        return;
    }
    // 生成一段数据DMA链表
    for (x = 0; x < lcd_cam_obj->lcd.dma_node_cnt; x++) {
        lcd_cam_obj->lcd.dma[x].size = lcd_cam_obj->lcd.dma_node_buffer_size;
        lcd_cam_obj->lcd.dma[x].length = lcd_cam_obj->lcd.dma_node_buffer_size;
        lcd_cam_obj->lcd.dma[x].buf = (lcd_cam_obj->lcd.dma_buffer + lcd_cam_obj->lcd.dma_node_buffer_size * x);
        lcd_cam_obj->lcd.dma[x].eof = !((x + 1) % lcd_cam_obj->lcd.dma_half_node_cnt);
        lcd_cam_obj->lcd.dma[x].empty = &lcd_cam_obj->lcd.dma[(x + 1) % lcd_cam_obj->lcd.dma_node_cnt];
    }
    lcd_cam_obj->lcd.dma[lcd_cam_obj->lcd.dma_half_node_cnt - 1].empty = NULL;
    lcd_cam_obj->lcd.dma[lcd_cam_obj->lcd.dma_node_cnt - 1].empty = NULL;
    cnt = len / lcd_cam_obj->lcd.dma_half_buffer_size;
    // 启动信号
    xQueueSend(lcd_cam_obj->lcd.event_queue, &event, 0);
    // 处理完整一段数据， 乒乓操作
    for (x = 0; x < cnt; x++) {
        memcpy(lcd_cam_obj->lcd.dma[(x % 2) * lcd_cam_obj->lcd.dma_half_node_cnt].buf, data, lcd_cam_obj->lcd.dma_half_buffer_size);
        data += lcd_cam_obj->lcd.dma_half_buffer_size;
        xQueueReceive(lcd_cam_obj->lcd.event_queue, (void *)&event, portMAX_DELAY);
        while (!I2S0.state.tx_idle);
        I2S0.conf.tx_reset = 1;
        I2S0.conf.tx_reset = 0;
        I2S0.conf.tx_fifo_reset = 1;
        I2S0.conf.tx_fifo_reset = 0;
        I2S0.out_link.addr = ((uint32_t)&lcd_cam_obj->lcd.dma[(x % 2) * lcd_cam_obj->lcd.dma_half_node_cnt]) & 0xfffff;
        I2S0.out_link.start = 1;
        ets_delay_us(1);
        I2S0.conf.tx_start = 1;
    }
    cnt = len % lcd_cam_obj->lcd.dma_half_buffer_size;
    // 处理剩余非完整段数据
    if (cnt) {
        memcpy(lcd_cam_obj->lcd.dma[(x % 2) * lcd_cam_obj->lcd.dma_half_node_cnt].buf, data, cnt);
        // 处理数据长度为 lcd_cam_obj->lcd.dma_node_buffer_size 的整数倍情况
        if (cnt % lcd_cam_obj->lcd.dma_node_buffer_size) {
            end_pos = (x % 2) * lcd_cam_obj->lcd.dma_half_node_cnt + cnt / lcd_cam_obj->lcd.dma_node_buffer_size;
            size = cnt % lcd_cam_obj->lcd.dma_node_buffer_size;
        } else {
            end_pos = (x % 2) * lcd_cam_obj->lcd.dma_half_node_cnt + cnt / lcd_cam_obj->lcd.dma_node_buffer_size - 1;
            size = lcd_cam_obj->lcd.dma_node_buffer_size;
        }
        // 处理尾节点，使其成为 DMA 尾
        lcd_cam_obj->lcd.dma[end_pos].size = size;
        lcd_cam_obj->lcd.dma[end_pos].length = size;
        lcd_cam_obj->lcd.dma[end_pos].eof = 1;
        lcd_cam_obj->lcd.dma[end_pos].empty = NULL;
        xQueueReceive(lcd_cam_obj->lcd.event_queue, (void *)&event, portMAX_DELAY);
        while (!I2S0.state.tx_idle);
        I2S0.conf.tx_reset = 1;
        I2S0.conf.tx_reset = 0;
        I2S0.conf.tx_fifo_reset = 1;
        I2S0.conf.tx_fifo_reset = 0;
        I2S0.out_link.addr = ((uint32_t)&lcd_cam_obj->lcd.dma[(x % 2) * lcd_cam_obj->lcd.dma_half_node_cnt]) & 0xfffff;
        I2S0.out_link.start = 1;
        ets_delay_us(1);
        I2S0.conf.tx_start = 1;
    }
    xQueueReceive(lcd_cam_obj->lcd.event_queue, (void *)&event, portMAX_DELAY);
}

static void lcd_cam_config(lcd_cam_config_t *config)
{
   // 配置时钟
    I2S0.clkm_conf.clkm_div_num = 2; // 160MHz / 2 = 80MHz
    I2S0.clkm_conf.clkm_div_b = 0;
    I2S0.clkm_conf.clkm_div_a = 0;
    I2S0.clkm_conf.clk_sel = 2;
    I2S0.clkm_conf.clk_en = 1;

    // 配置采样率
    I2S0.sample_rate_conf.tx_bck_div_num = 40000000 / config->lcd.fre; // Fws = Fbck / 2
    I2S0.sample_rate_conf.tx_bits_mod = config->lcd.width;
    I2S0.sample_rate_conf.rx_bck_div_num = 1;
    I2S0.sample_rate_conf.rx_bits_mod = config->cam.width;

    // 配置数据格式
    I2S0.conf.val = 0;
    I2S0.conf.tx_right_first = 1;
    I2S0.conf.tx_msb_right = 1;
    I2S0.conf.tx_dma_equal = 1;
    I2S0.conf.rx_slave_mod = 1;
    I2S0.conf.rx_right_first = 0;
    I2S0.conf.rx_msb_right = 0;
    I2S0.conf.rx_short_sync = 0;
    I2S0.conf.rx_mono = 0;
    I2S0.conf.rx_msb_shift = 0;
    I2S0.conf.rx_dma_equal = 1;

    I2S0.conf1.tx_pcm_bypass = 1;
    I2S0.conf1.tx_stop_en = 1;
    I2S0.conf1.rx_pcm_bypass = 1;

    I2S0.conf2.cam_sync_fifo_reset = 1;
    I2S0.conf2.cam_sync_fifo_reset = 0;
    I2S0.conf2.lcd_en = 1;
    I2S0.conf2.camera_en = 1;
    I2S0.conf2.i_v_sync_filter_en = 1;
    I2S0.conf2.i_v_sync_filter_thres = 1;

    I2S0.conf_chan.tx_chan_mod = 1;
    I2S0.conf_chan.rx_chan_mod = 1;

    I2S0.fifo_conf.val = 0;
    I2S0.fifo_conf.tx_fifo_mod_force_en = 1;
    I2S0.fifo_conf.tx_data_num = 32;
    I2S0.fifo_conf.tx_fifo_mod = 2;
    I2S0.fifo_conf.tx_24msb_en = 0;
    I2S0.fifo_conf.rx_fifo_mod_force_en = 1;
    I2S0.fifo_conf.rx_data_num = 32;
    I2S0.fifo_conf.rx_fifo_mod = 2;
    I2S0.fifo_conf.dscr_en = 1;

    I2S0.lc_conf.out_rst  = 1;
    I2S0.lc_conf.out_rst  = 0;
    I2S0.lc_conf.in_rst  = 1;
    I2S0.lc_conf.in_rst  = 0;
    I2S0.lc_conf.ahbm_fifo_rst = 1;
    I2S0.lc_conf.ahbm_fifo_rst = 0;
    I2S0.lc_conf.ahbm_rst = 1;
    I2S0.lc_conf.ahbm_rst = 0;
    I2S0.lc_conf.check_owner = 0;

    I2S0.timing.val = 0;

    I2S0.int_ena.val = 0;
    I2S0.int_clr.val = ~0;
    I2S0.int_ena.out_eof = 1;
    I2S0.conf.rx_start = 1;

    ESP_LOGI(TAG, "I2S version  %x\n", I2S0.date);
}

static void lcd_set_pin(lcd_config_t *config)
{
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin.clk], PIN_FUNC_GPIO);
    gpio_set_direction(config->pin.clk, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(config->pin.clk, GPIO_FLOATING);
    gpio_matrix_out(config->pin.clk, I2S0O_WS_OUT_IDX, true, config->invert.clk);

    for(int i = 0; i < config->width; i++) {
        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin.data[i]], PIN_FUNC_GPIO);
        gpio_set_direction(config->pin.data[i], GPIO_MODE_OUTPUT);
        gpio_set_pull_mode(config->pin.data[i], GPIO_FLOATING);
        // 高位对齐，OUT23总是最高位
        // fifo按bit来访问数据，tx_bits_mod为8时，数据需要按8位对齐
        gpio_matrix_out(config->pin.data[i], I2S0O_DATA_OUT0_IDX + (24 - config->width) + i, false, config->invert.data[i]);
    }
}

void lcd_dma_config(lcd_config_t *config) 
{
    int cnt = 0;
    if (config->max_buffer_size >= LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE * 2) {
        lcd_cam_obj->lcd.dma_node_buffer_size = (LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE / (config->width >> 3)) * (config->width >> 3);
        for (cnt = 0;;cnt++) { // 寻找可以整除dma_size的buffer大小
            if ((config->max_buffer_size - cnt) % (lcd_cam_obj->lcd.dma_node_buffer_size * 2) == 0) {
                break;
            }
        }
        lcd_cam_obj->lcd.dma_buffer_size = config->max_buffer_size - cnt;
    } else {
        lcd_cam_obj->lcd.dma_node_buffer_size = config->max_buffer_size / 2;
        lcd_cam_obj->lcd.dma_buffer_size = lcd_cam_obj->lcd.dma_node_buffer_size * 2;
    }
    
    lcd_cam_obj->lcd.dma_half_buffer_size = lcd_cam_obj->lcd.dma_buffer_size / 2;

    lcd_cam_obj->lcd.dma_node_cnt = (lcd_cam_obj->lcd.dma_buffer_size) / lcd_cam_obj->lcd.dma_node_buffer_size; // DMA节点个数
    lcd_cam_obj->lcd.dma_half_node_cnt = lcd_cam_obj->lcd.dma_node_cnt / 2;

    ESP_LOGI(TAG, "lcd_buffer_size: %d, lcd_dma_size: %d, lcd_dma_node_cnt: %d\n", lcd_cam_obj->lcd.dma_buffer_size, lcd_cam_obj->lcd.dma_node_buffer_size, lcd_cam_obj->lcd.dma_node_cnt);

    lcd_cam_obj->lcd.dma    = (lldesc_t *)heap_caps_malloc(lcd_cam_obj->lcd.dma_node_cnt * sizeof(lldesc_t), MALLOC_CAP_DMA);
    lcd_cam_obj->lcd.dma_buffer = (uint8_t *)heap_caps_malloc(lcd_cam_obj->lcd.dma_buffer_size * sizeof(uint8_t), MALLOC_CAP_DMA);
}

#include "hal/gpio_ll.h"
void IRAM_ATTR cam_vsync_isr(void *arg)
{
    cam_event_t cam_event = {0};
    BaseType_t HPTaskAwoken = pdFALSE;
    // filter
    ets_delay_us(1);
    if (gpio_ll_get_level(&GPIO, lcd_cam_obj->cam.vsync_pin) == !lcd_cam_obj->cam.vsync_invert) {
        cam_event = CAM_VSYNC_EVENT;
        xQueueSendFromISR(lcd_cam_obj->cam.event_queue, (void *)&cam_event, &HPTaskAwoken);
    }

    if(HPTaskAwoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void cam_vsync_intr_enable(uint8_t en)
{
    if (en) {
        gpio_intr_enable(lcd_cam_obj->cam.vsync_pin);
    } else {
        gpio_intr_disable(lcd_cam_obj->cam.vsync_pin);
    }
}

static void cam_dma_stop(void)
{
    if (I2S0.int_ena.in_suc_eof == 1) {
        I2S0.conf.rx_start = 0;
        I2S0.int_ena.in_suc_eof = 0;
        I2S0.int_clr.in_suc_eof = 1;
        I2S0.in_link.stop = 1;
    }
}

static void cam_dma_start(void)
{
    if (I2S0.int_ena.in_suc_eof == 0) {
        I2S0.conf.rx_start = 0;
        I2S0.int_clr.in_suc_eof = 1;
        I2S0.int_ena.in_suc_eof = 1;
        I2S0.conf.rx_reset = 1;
        I2S0.conf.rx_reset = 0;
        I2S0.conf.rx_fifo_reset = 1;
        I2S0.conf.rx_fifo_reset = 0;
        I2S0.lc_conf.in_rst = 1;
        I2S0.lc_conf.in_rst = 0;
        I2S0.lc_conf.ahbm_fifo_rst = 1;
        I2S0.lc_conf.ahbm_fifo_rst = 0;
        I2S0.lc_conf.ahbm_rst = 1;
        I2S0.lc_conf.ahbm_rst = 0;
        I2S0.in_link.start = 1;
        I2S0.conf.rx_start = 1;
        if(lcd_cam_obj->cam.jpeg_mode) {
            // 手动给第一帧vsync
            gpio_matrix_in(lcd_cam_obj->cam.vsync_pin, I2S0I_V_SYNC_IDX, !lcd_cam_obj->cam.vsync_invert);
            gpio_matrix_in(lcd_cam_obj->cam.vsync_pin, I2S0I_V_SYNC_IDX, lcd_cam_obj->cam.vsync_invert);
        }
    }
}

void cam_stop(void)
{
    cam_vsync_intr_enable(0);
    cam_dma_stop();
}

void cam_start(void)
{
    cam_vsync_intr_enable(1);
}

typedef enum {
    CAM_STATE_IDLE = 0,
    CAM_STATE_READ_BUF = 1,
} cam_state_t;

//Copy fram from DMA dma_buffer to fram dma_buffer
static void cam_task(void *arg)
{
    int cnt = 0;
    int frame_pos = 0;
    int state = CAM_STATE_IDLE;
    cam_event_t cam_event = {0};
    frame_buffer_event_t frame_buffer_event = {0};
    xQueueReset(lcd_cam_obj->cam.event_queue);
    while (1) {
        xQueueReceive(lcd_cam_obj->cam.event_queue, (void *)&cam_event, portMAX_DELAY);
        switch (state) {
            case CAM_STATE_IDLE: {
                if (cam_event == CAM_VSYNC_EVENT) { 
                    for (int x = 0; x < lcd_cam_obj->cam.frame_cnt; x++) {
                        if (lcd_cam_obj->cam.frame_en[x]) {
                            frame_pos = x;
                            cam_dma_start();
                            cam_vsync_intr_enable(0);
                            state = CAM_STATE_READ_BUF;
                            break;
                        }
                    }
                    cnt = 0;
                }
            }
            break;

            case CAM_STATE_READ_BUF: {
                if (cam_event == CAM_IN_SUC_EOF_EVENT) {
                    if (cnt == 0) {
                        cam_vsync_intr_enable(1); // 需要cam真正start接收到第一个buf数据再打开vsync中断
                    }
                    uint8_t *out_buf = &lcd_cam_obj->cam.frame_buffer[(frame_pos * lcd_cam_obj->cam.recv_size) + cnt * lcd_cam_obj->cam.dma_half_buffer_size];
                    uint8_t *in_buf = &lcd_cam_obj->cam.dma_buffer[(cnt % 2) * lcd_cam_obj->cam.dma_half_buffer_size];
                    memcpy(out_buf, in_buf, lcd_cam_obj->cam.dma_half_buffer_size);

                    if(lcd_cam_obj->cam.jpeg_mode) {
                        if (lcd_cam_obj->cam.frame_en[frame_pos] == 0) {
                            cam_dma_stop();
                        }
                    } else {
                        if(cnt == lcd_cam_obj->cam.frame_copy_cnt - 1) {
                            lcd_cam_obj->cam.frame_en[frame_pos] = 0;
                        }
                    }

                    if(lcd_cam_obj->cam.frame_en[frame_pos] == 0) {
                        frame_buffer_event.frame_buffer = &lcd_cam_obj->cam.frame_buffer[frame_pos * lcd_cam_obj->cam.recv_size];
                        frame_buffer_event.len = (cnt + 1) * lcd_cam_obj->cam.dma_half_buffer_size;
                        xQueueSend(lcd_cam_obj->cam.frame_buffer_queue, (void *)&frame_buffer_event, portMAX_DELAY);
                        state = CAM_STATE_IDLE;
                    } else {
                        cnt++;
                    }
                } else if (cam_event == CAM_VSYNC_EVENT) {
                    if(lcd_cam_obj->cam.jpeg_mode) {
                        lcd_cam_obj->cam.frame_en[frame_pos] = 0;
                    }
                }
            }
            break;
        }
    }
}

size_t cam_take(uint8_t **buffer_p)
{
    frame_buffer_event_t frame_buffer_event;
    xQueueReceive(lcd_cam_obj->cam.frame_buffer_queue, (void *)&frame_buffer_event, portMAX_DELAY);
    *buffer_p = frame_buffer_event.frame_buffer;
    return frame_buffer_event.len;
}

void cam_give(uint8_t *dma_buffer)
{
    for (int x = 0; x < lcd_cam_obj->cam.frame_cnt; x++) {
        if (&lcd_cam_obj->cam.frame_buffer[x * lcd_cam_obj->cam.recv_size] == dma_buffer) {
            lcd_cam_obj->cam.frame_en[x] = 1;
            break;
        }
    }
}

static void cam_set_pin(cam_config_t *config)
{
    gpio_config_t io_conf = {0};
    io_conf.intr_type = config->invert.vsync ? GPIO_PIN_INTR_NEGEDGE : GPIO_PIN_INTR_POSEDGE;
    io_conf.pin_bit_mask = 1 << config->pin.vsync; 
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);
    gpio_install_isr_service(ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(config->pin.vsync, cam_vsync_isr, NULL);
    gpio_intr_disable(config->pin.vsync);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin.pclk], PIN_FUNC_GPIO);
    gpio_set_direction(config->pin.pclk, GPIO_MODE_INPUT);
    gpio_set_pull_mode(config->pin.pclk, GPIO_FLOATING);
    gpio_matrix_in(config->pin.pclk, I2S0I_WS_IN_IDX, config->invert.pclk);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin.vsync], PIN_FUNC_GPIO);
    gpio_set_direction(config->pin.vsync, GPIO_MODE_INPUT);
    gpio_set_pull_mode(config->pin.vsync, GPIO_FLOATING);
    gpio_matrix_in(config->pin.vsync, I2S0I_V_SYNC_IDX, config->invert.vsync);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin.href], PIN_FUNC_GPIO);
    gpio_set_direction(config->pin.href, GPIO_MODE_INPUT);
    gpio_set_pull_mode(config->pin.href, GPIO_FLOATING);
    gpio_matrix_in(config->pin.href, I2S0I_H_SYNC_IDX, config->invert.href);

    for(int i = 0; i < config->width; i++) {
        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin.data[i]], PIN_FUNC_GPIO);
        gpio_set_direction(config->pin.data[i], GPIO_MODE_INPUT);
        gpio_set_pull_mode(config->pin.data[i], GPIO_FLOATING);
        // 高位对齐，IN16总是最高位
        // fifo按bit来访问数据，rx_bits_mod为8时，数据需要按8位对齐
        gpio_matrix_in(config->pin.data[i], I2S0I_DATA_IN0_IDX + (16 - config->width) + i, config->invert.data[i]);
    }

    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_1_BIT,
        .freq_hz = config->fre,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_1
    };
    ledc_timer_config(&ledc_timer);
    ledc_channel_config_t ledc_channel = {
        .channel    = LEDC_CHANNEL_2,
        .duty       = 1,
        .gpio_num   = config->pin.xclk,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel  = LEDC_TIMER_1,
        .hpoint     = 0
    };
    ledc_channel_config(&ledc_channel);

    gpio_matrix_in(0x38, I2S0I_H_ENABLE_IDX, false);
    ESP_LOGI(TAG, "cam_xclk_pin setup\n");
}

void cam_dma_config(cam_config_t *config) 
{
    int cnt = 0;
    if (config->mode.jpeg) {
        lcd_cam_obj->cam.dma_buffer_size = 8000;
        lcd_cam_obj->cam.dma_half_buffer_size = lcd_cam_obj->cam.dma_buffer_size / 2;
        lcd_cam_obj->cam.dma_node_buffer_size = 4000;
    } else {
        if (config->max_dma_buffer_size / 2.0 > 16384) { // must less than max(cam_rec_data_bytelen)
            config->max_dma_buffer_size = 16384 * 2;
        }
        for (cnt = 0;;cnt++) { // 寻找可以整除的buffer大小
            if (lcd_cam_obj->cam.recv_size % (config->max_dma_buffer_size - cnt) == 0) {
                break;
            }
        }
        lcd_cam_obj->cam.dma_buffer_size = config->max_dma_buffer_size - cnt;

        lcd_cam_obj->cam.dma_half_buffer_size = lcd_cam_obj->cam.dma_buffer_size / 2;
        for (cnt = 0;;cnt++) { // 寻找可以整除的dma大小
            if ((lcd_cam_obj->cam.dma_half_buffer_size) % (LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE - cnt) == 0) {
                break;
            }
        }
        lcd_cam_obj->cam.dma_node_buffer_size = LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE - cnt;
    }

    lcd_cam_obj->cam.dma_node_cnt = (lcd_cam_obj->cam.dma_buffer_size) / lcd_cam_obj->cam.dma_node_buffer_size; // DMA节点个数
    lcd_cam_obj->cam.frame_copy_cnt = lcd_cam_obj->cam.recv_size / lcd_cam_obj->cam.dma_half_buffer_size; // 产生中断拷贝的次数, 乒乓拷贝

    ESP_LOGI(TAG, "cam_buffer_size: %d, cam_dma_size: %d, cam_dma_node_cnt: %d, cam_total_cnt: %d\n", lcd_cam_obj->cam.dma_buffer_size, lcd_cam_obj->cam.dma_node_buffer_size, lcd_cam_obj->cam.dma_node_cnt, lcd_cam_obj->cam.frame_copy_cnt);

    lcd_cam_obj->cam.dma    = (lldesc_t *)heap_caps_malloc(lcd_cam_obj->cam.dma_node_cnt * sizeof(lldesc_t), MALLOC_CAP_DMA);
    lcd_cam_obj->cam.dma_buffer = (uint8_t *)heap_caps_malloc(lcd_cam_obj->cam.dma_buffer_size * sizeof(uint8_t), MALLOC_CAP_DMA);

    for (int x = 0; x < lcd_cam_obj->cam.dma_node_cnt; x++) {
        lcd_cam_obj->cam.dma[x].size = lcd_cam_obj->cam.dma_node_buffer_size;
        lcd_cam_obj->cam.dma[x].length = lcd_cam_obj->cam.dma_node_buffer_size;
        lcd_cam_obj->cam.dma[x].eof = 0;
        lcd_cam_obj->cam.dma[x].owner = 1;
        lcd_cam_obj->cam.dma[x].buf = (lcd_cam_obj->cam.dma_buffer + lcd_cam_obj->cam.dma_node_buffer_size * x);
        lcd_cam_obj->cam.dma[x].empty = &lcd_cam_obj->cam.dma[(x + 1) % lcd_cam_obj->cam.dma_node_cnt];
    }

    I2S0.in_link.addr = ((uint32_t)&lcd_cam_obj->cam.dma[0]) & 0xfffff;
    I2S0.rx_eof_num = lcd_cam_obj->cam.dma_half_buffer_size - 1; // 乒乓操作
}

int lcd_cam_init(lcd_cam_config_t *config)
{
    lcd_cam_obj = (lcd_cam_obj_t *)heap_caps_calloc(1, sizeof(lcd_cam_obj_t), MALLOC_CAP_DMA);
    if (!lcd_cam_obj) {
        ESP_LOGI(TAG, "lcd_cam object malloc error\n");
        return -1;
    }

    //Enable I2S periph
    periph_module_enable(PERIPH_I2S0_MODULE);
    lcd_cam_config(config);
    esp_intr_alloc(ETS_I2S0_INTR_SOURCE, ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM, lcd_cam_isr, NULL, &lcd_cam_obj->intr_handle);

    if (config->lcd.en) {
        lcd_set_pin(&config->lcd);
        lcd_dma_config(&config->lcd);

        lcd_cam_obj->lcd.event_queue = xQueueCreate(1, sizeof(int));
        lcd_cam_obj->lcd.dma_buffer_size = config->lcd.max_buffer_size;
        ESP_LOGI(TAG, "lcd init ok\n");
    }

    if (config->cam.en) {
        lcd_cam_obj->cam.jpeg_mode = config->cam.mode.jpeg;
        lcd_cam_obj->cam.vsync_pin = config->cam.pin.vsync;
        lcd_cam_obj->cam.vsync_invert = config->cam.invert.vsync;
        lcd_cam_obj->cam.frame_cnt = config->cam.frame_cnt;
        lcd_cam_obj->cam.recv_size = config->cam.recv_size;
        cam_set_pin(&config->cam);
        cam_dma_config(&config->cam);

        lcd_cam_obj->cam.event_queue = xQueueCreate(2, sizeof(cam_event_t));
        lcd_cam_obj->cam.frame_buffer_queue = xQueueCreate(lcd_cam_obj->cam.frame_cnt, sizeof(frame_buffer_event_t));
        lcd_cam_obj->cam.frame_buffer = (uint8_t *)heap_caps_malloc(lcd_cam_obj->cam.frame_cnt * lcd_cam_obj->cam.recv_size * sizeof(uint8_t), config->cam.frame_caps);
        lcd_cam_obj->cam.frame_en = (uint8_t *)heap_caps_malloc(lcd_cam_obj->cam.frame_cnt * sizeof(uint8_t), MALLOC_CAP_DEFAULT);

        for (int x = 0; x < lcd_cam_obj->cam.frame_cnt; x++) {
            lcd_cam_obj->cam.frame_en[x] = 1;
        }
        xTaskCreate(cam_task, "cam_task", config->cam.task_stack, NULL, config->cam.task_pri, &lcd_cam_obj->cam.task_handle);
        ESP_LOGI(TAG, "cam init ok\n");
    }
    
    return 0;
}
