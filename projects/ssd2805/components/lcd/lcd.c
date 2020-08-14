#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp32s2/rom/lldesc.h"
#include "soc/system_reg.h"
#include "esp_log.h"
#include "lcd.h"

static const char *TAG = "lcd";

#define LCD_DMA_MAX_SIZE     (4095)

typedef struct {
    uint32_t buffer_size;
    uint32_t half_buffer_size;
    uint32_t node_cnt;
    uint32_t half_node_cnt;
    uint32_t dma_size;
    uint8_t horizontal;
    uint8_t dc_state;
    uint8_t bit_width;
    int8_t pin_dc;
    int8_t pin_cs;
    int8_t pin_rst;
    int8_t pin_bk;
    lldesc_t *dma;
    uint8_t *buffer;
    QueueHandle_t event_queue;
} lcd_obj_t;

static lcd_obj_t *lcd_obj = NULL;

void inline lcd_set_rst(uint8_t state)
{
    if (lcd_obj->pin_rst < 0) {
        return;
    }
    gpio_set_level(lcd_obj->pin_rst, state);
}

void inline lcd_set_dc(uint8_t state)
{
    gpio_set_level(lcd_obj->pin_dc, state);
}

void inline lcd_set_cs(uint8_t state)
{
    if (lcd_obj->pin_cs < 0) {
        return;
    }
    gpio_set_level(lcd_obj->pin_cs, state);
}

void inline lcd_set_blk(uint8_t state)
{
    if (lcd_obj->pin_bk < 0) {
        return;
    }
    gpio_set_level(lcd_obj->pin_bk, state);
}

static void IRAM_ATTR lcd_isr(void *arg)
{
    int event;
    BaseType_t HPTaskAwoken = pdFALSE;
    typeof(I2S0.int_st) int_st = I2S0.int_st;
    I2S0.int_clr.val = int_st.val;
    // ets_printf("intr: 0x%x\n", int_st);

    if (int_st.out_eof) {
        xQueueSendFromISR(lcd_obj->event_queue, &int_st.val, &HPTaskAwoken);
    }

    if (HPTaskAwoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void i2s_write_data(uint8_t *data, size_t len)
{
    int event  = 0;
    int x = 0, cnt = 0, size = 0;
    int end_pos = 0;
    lcd_set_cs(0);
    lcd_set_dc(lcd_obj->dc_state);
    // 生成一段数据DMA链表
    for (x = 0; x < lcd_obj->node_cnt; x++) {
        lcd_obj->dma[x].size = lcd_obj->dma_size;
        lcd_obj->dma[x].length = lcd_obj->dma_size;
        lcd_obj->dma[x].buf = (lcd_obj->buffer + lcd_obj->dma_size * x);
        lcd_obj->dma[x].eof = !((x + 1) % lcd_obj->half_node_cnt);
        lcd_obj->dma[x].empty = &lcd_obj->dma[(x + 1) % lcd_obj->node_cnt];
    }
    lcd_obj->dma[lcd_obj->half_node_cnt - 1].empty = NULL;
    lcd_obj->dma[lcd_obj->node_cnt - 1].empty = NULL;
    cnt = len / lcd_obj->half_buffer_size;
    // 启动信号
    xQueueSend(lcd_obj->event_queue, &event, 0);
    // 处理完整一段数据， 乒乓操作
    for (x = 0; x < cnt; x++) {
        memcpy(lcd_obj->dma[(x % 2) * lcd_obj->half_node_cnt].buf, data, lcd_obj->half_buffer_size);
        data += lcd_obj->half_buffer_size;
        xQueueReceive(lcd_obj->event_queue, (void *)&event, portMAX_DELAY);
        I2S0.conf.tx_reset = 1;
        I2S0.conf.tx_reset = 0;
        I2S0.conf.tx_fifo_reset = 1;
        I2S0.conf.tx_fifo_reset = 0;
        I2S0.out_link.addr = ((uint32_t)&lcd_obj->dma[(x % 2) * lcd_obj->half_node_cnt]) & 0xfffff;
        I2S0.out_link.start = 1;
        ets_delay_us(1);
        I2S0.conf.tx_start = 1;
    }
    cnt = len % lcd_obj->half_buffer_size;
    // 处理剩余非完整段数据
    if (cnt) {
        memcpy(lcd_obj->dma[(x % 2) * lcd_obj->half_node_cnt].buf, data, cnt);
        // 处理数据长度为 lcd_obj->dma_size 的整数倍情况
        if (cnt % lcd_obj->dma_size) {
            end_pos = (x % 2) * lcd_obj->half_node_cnt + cnt / lcd_obj->dma_size;
            size = cnt % lcd_obj->dma_size;
        } else {
            end_pos = (x % 2) * lcd_obj->half_node_cnt + cnt / lcd_obj->dma_size - 1;
            size = lcd_obj->dma_size;
        }
        // 处理尾节点，使其成为 DMA 尾
        lcd_obj->dma[end_pos].size = size;
        lcd_obj->dma[end_pos].length = size;
        lcd_obj->dma[end_pos].eof = 1;
        lcd_obj->dma[end_pos].empty = NULL;
        xQueueReceive(lcd_obj->event_queue, (void *)&event, portMAX_DELAY);
        I2S0.conf.tx_reset = 1;
        I2S0.conf.tx_reset = 0;
        I2S0.conf.tx_fifo_reset = 1;
        I2S0.conf.tx_fifo_reset = 0;
        I2S0.out_link.addr = ((uint32_t)&lcd_obj->dma[(x % 2) * lcd_obj->half_node_cnt]) & 0xfffff;
        I2S0.out_link.start = 1;
        ets_delay_us(1);
        I2S0.conf.tx_start = 1;
    }
    xQueueReceive(lcd_obj->event_queue, (void *)&event, portMAX_DELAY);
    lcd_set_cs(1);
}

static void lcd_delay_ms(uint32_t time)
{
    vTaskDelay(time / portTICK_RATE_MS);
}

static void lcd_write_cmd(uint8_t data)
{
    lcd_obj->dc_state = 0;
    i2s_write_data(&data, 1);
}

static void lcd_write_reg(uint8_t data)
{
    lcd_obj->dc_state = 1;
    i2s_write_data(&data, 1);
}

void lcd_write_data(uint8_t *data, size_t len)
{
    if (len <= 0) {
        return;
    }
    lcd_obj->dc_state = 1;
    i2s_write_data(data, len);
}

void lcd_rst()
{
    lcd_set_rst(0);
    lcd_delay_ms(100);
    lcd_set_rst(1);
    lcd_delay_ms(100);
}

void ssd2805_write_reg(uint8_t cmd, uint16_t data)
{
    lcd_write_cmd(cmd);
    lcd_write_reg(data & 0xff);
    lcd_write_reg(data >> 8);
}
void gp_commad_pa(uint16_t pa)
{
    ssd2805_write_reg(0xBC, pa);
    ssd2805_write_reg(0xBD, 0x0000);
    lcd_write_cmd(0xBF);
}

void dcs_packet_trans(void)
{
    ssd2805_write_reg(0xBD, 0x0004);
    ssd2805_write_reg(0xBC, 0xb000);
    ssd2805_write_reg(0xB7, 0x0150);
    lcd_write_cmd(0x2c);
}

void lcd_ssd2805_config(lcd_config_t *config)
{
    ssd2805_write_reg(0xB9, 0x0000); //PLL Control, Disable
    lcd_delay_ms(20);
    ssd2805_write_reg(0xBA, 0x0012); //PLL Configuration, P & N M, P = 0x1, N = 0x1, M = 0x12
    ssd2805_write_reg(0xB9, 0x0001); //PLL Control, Enable
    lcd_delay_ms(20);
    ssd2805_write_reg(0xD6, 0x0105); //Test, R2 PNB & END & COLOR
    ssd2805_write_reg(0xB8, 0x0000); //Virtual Channel Control
    ssd2805_write_reg(0xBB, 0x0003); //Clock Control, SYS_CLK & LP Clock
    lcd_delay_ms(25);
}

void lcd_config_lcm(lcd_config_t *config)
{
    ssd2805_write_reg(0xB7,0x0210); //Generic Packet

    gp_commad_pa(1);
    lcd_write_reg(0x11); // Sleep Out
    lcd_delay_ms(120);
    gp_commad_pa(2);
    lcd_write_reg(0xf0) ;
    lcd_write_reg(0xc3) ;
    gp_commad_pa(2);
    lcd_write_reg(0xf0) ;
    lcd_write_reg(0x96) ;
    gp_commad_pa(2);
    lcd_write_reg(0x36);
    lcd_write_reg(0x48);
    gp_commad_pa(2);
    lcd_write_reg(0x3A);
    lcd_write_reg(0x77);
    gp_commad_pa(2);
    lcd_write_reg(0xB4);
    lcd_write_reg(0x01);
    gp_commad_pa(2);
    lcd_write_reg(0xB7) ;
    lcd_write_reg(0xC6) ;

    gp_commad_pa(9);
    lcd_write_reg(0xe8);
    lcd_write_reg(0x40);
    lcd_write_reg(0x8a);
    lcd_write_reg(0x00);
    lcd_write_reg(0x00);
    lcd_write_reg(0x29);
    lcd_write_reg(0x19);
    lcd_write_reg(0xa5);
    lcd_write_reg(0x33);
    gp_commad_pa(2);
    lcd_write_reg(0xc1);
    lcd_write_reg(0x06);
    gp_commad_pa(2);
    lcd_write_reg(0xc2);
    lcd_write_reg(0xa7);
    gp_commad_pa(2);
    lcd_write_reg(0xc5);
    lcd_write_reg(0x18);
    gp_commad_pa(15);
    lcd_write_reg(0xe0); //Positive Voltage Gamma Control
    lcd_write_reg(0xf0);
    lcd_write_reg(0x09);
    lcd_write_reg(0x0b);
    lcd_write_reg(0x06);
    lcd_write_reg(0x04);
    lcd_write_reg(0x15);
    lcd_write_reg(0x2f);
    lcd_write_reg(0x54);
    lcd_write_reg(0x42);
    lcd_write_reg(0x3c);
    lcd_write_reg(0x17);
    lcd_write_reg(0x14);
    lcd_write_reg(0x18);
    lcd_write_reg(0x1b);

    gp_commad_pa(15);
    lcd_write_reg(0xe1); //Negative Voltage Gamma Control
    lcd_write_reg(0xf0);
    lcd_write_reg(0x09);
    lcd_write_reg(0x0b);
    lcd_write_reg(0x06);
    lcd_write_reg(0x04);
    lcd_write_reg(0x03);
    lcd_write_reg(0x2d);
    lcd_write_reg(0x43);
    lcd_write_reg(0x42);
    lcd_write_reg(0x3b);
    lcd_write_reg(0x16);
    lcd_write_reg(0x14);
    lcd_write_reg(0x17);
    lcd_write_reg(0x1b);
    gp_commad_pa(2);
    lcd_write_reg(0xf0);
    lcd_write_reg(0x3c);
    gp_commad_pa(2);
    lcd_write_reg(0xf0);
    lcd_write_reg(0x69);
    lcd_delay_ms(120);
    gp_commad_pa(1);
    lcd_write_reg(0x29); 

    // Reverse display
    ssd2805_write_reg(0xB7, 0x0250);
    ssd2805_write_reg(0xBD, 0x0000);
    ssd2805_write_reg(0xBC, 0x0000);
    lcd_write_cmd(0x20);

    ssd2805_write_reg(0xB7, 0x0250);
    ssd2805_write_reg(0xBD, 0x0000);
    ssd2805_write_reg(0xBC, 0x0001);
    lcd_write_cmd(0x36); 
    lcd_write_reg(0x00);

    ssd2805_write_reg(0xB7, 0x0250);
    ssd2805_write_reg(0xBD, 0x0000);
    ssd2805_write_reg(0xBC, 0x0001);
    lcd_write_cmd(0x3A); 
    lcd_write_reg(0x55);
    //

    ssd2805_write_reg(0xB7, 0x0250);
    ssd2805_write_reg(0xBD, 0x0000);
    ssd2805_write_reg(0xBC, 0x0000);
    lcd_write_cmd(0x21); 

    //
    ssd2805_write_reg(0xB7, 0x0250);
    ssd2805_write_reg(0xBD, 0x0000);
    ssd2805_write_reg(0xBC, 0x0000);
    lcd_write_cmd(0x11); 

    ssd2805_write_reg(0xB7, 0x0250);
    ssd2805_write_reg(0xBD, 0x0000);
    ssd2805_write_reg(0xBC, 0x0000);
    lcd_write_cmd(0x29); 

    ssd2805_write_reg(0xBD, 0x0011);
    ssd2805_write_reg(0xBC, 0x9400);
    ssd2805_write_reg(0xBE, 0x02D0);
    lcd_write_cmd(0x2C); 
}

void lcd_set_index(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end)
{
	//ILI9481
    ssd2805_write_reg(0xB7,0x0210); //Generic Packet 
    gp_commad_pa(5); 
	lcd_write_reg(0x2a);   
	lcd_write_reg(x_start >> 8);
	lcd_write_reg(x_start & 0xff);
	lcd_write_reg(x_end >> 8);
	lcd_write_reg(x_end & 0xff);
    
    gp_commad_pa(5);   
    lcd_write_reg(0x2b);
	lcd_write_reg(y_start >> 8);
	lcd_write_reg(y_start & 0xff);
	lcd_write_reg(y_end >> 8);
	lcd_write_reg(y_end & 0xff);
    dcs_packet_trans();
}

static void lcd_config(lcd_config_t *config)
{
    //Enable I2S periph
    periph_module_enable(PERIPH_I2S0_MODULE);

   // 配置时钟
    I2S0.clkm_conf.clkm_div_num = 2; // 160MHz / 2 = 80MHz
    I2S0.clkm_conf.clkm_div_b = 0;
    I2S0.clkm_conf.clkm_div_a = 0;
    I2S0.clkm_conf.clk_sel = 2;
    I2S0.clkm_conf.clk_en = 1;

    // 配置采样率
    I2S0.sample_rate_conf.tx_bck_div_num = 40000000 / config->clk_fre; // Fws = Fbck / 2
    I2S0.sample_rate_conf.tx_bits_mod = config->bit_width;

    // 配置数据格式
    I2S0.conf.val = 0;
    I2S0.conf.tx_right_first = 1;
    I2S0.conf.tx_msb_right = 1;
    I2S0.conf.tx_dma_equal = 1;

    I2S0.conf1.tx_pcm_bypass = 1;
    I2S0.conf1.tx_stop_en = 1;
    I2S0.conf1.rx_pcm_bypass = 1;

    I2S0.conf2.lcd_en = 1;
    I2S0.conf2.camera_en = 1;

    I2S0.conf_chan.tx_chan_mod = 1;

    // I2S0.fifo_conf.val = 0;
    I2S0.fifo_conf.tx_fifo_mod_force_en = 1;
    I2S0.fifo_conf.tx_data_num = 32;
    I2S0.fifo_conf.tx_fifo_mod = 2;
    I2S0.fifo_conf.tx_24msb_en = 0;
    I2S0.fifo_conf.dscr_en = 1;

    I2S0.lc_conf.out_rst  = 1;
    I2S0.lc_conf.out_rst  = 0;

    I2S0.timing.val = 0;

    I2S0.int_ena.val = 0;
    I2S0.int_clr.val = ~0;

    I2S0.lc_conf.check_owner = 0;

    I2S0.int_ena.out_eof = 1;

    ESP_LOGI(TAG, "I2S version  %x\n", I2S0.date);
    esp_intr_alloc(ETS_I2S0_INTR_SOURCE, 0, lcd_isr, NULL, NULL);
}

static void lcd_set_pin(lcd_config_t *config)
{
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin_clk], PIN_FUNC_GPIO);
    gpio_set_direction(config->pin_clk, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(config->pin_clk, GPIO_FLOATING);
    gpio_matrix_out(config->pin_clk, I2S0O_WS_OUT_IDX, true, 0);

    for(int i = 0; i < config->bit_width; i++) {
        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin_data[i]], PIN_FUNC_GPIO);
        gpio_set_direction(config->pin_data[i], GPIO_MODE_OUTPUT);
        gpio_set_pull_mode(config->pin_data[i], GPIO_FLOATING);
        // 高位对齐，OUT23总是最高位
        // fifo按bit来访问数据，tx_bits_mod为8时，数据需要按8位对齐
        gpio_matrix_out(config->pin_data[i], I2S0O_DATA_OUT0_IDX + (24 - config->bit_width) + i, false, 0);
    }

    //Initialize non-matrix GPIOs
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask  = (config->pin_dc < 0) ? 0ULL : (1ULL << config->pin_dc);
    io_conf.pin_bit_mask |= (config->pin_rd < 0) ? 0ULL : (1ULL << config->pin_rd);
    io_conf.pin_bit_mask |= (config->pin_cs < 0) ? 0ULL : (1ULL << config->pin_cs);
    io_conf.pin_bit_mask |= (config->pin_rst < 0) ? 0ULL : (1ULL << config->pin_rst);
    io_conf.pin_bit_mask |= (config->pin_bk < 0) ? 0ULL : (1ULL << config->pin_bk);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(config->pin_rd, 1);
}

void lcd_dma_config(lcd_config_t *config) 
{
    int cnt = 0;
    if (config->max_buffer_size >= LCD_DMA_MAX_SIZE * 2) {
        lcd_obj->dma_size = LCD_DMA_MAX_SIZE;
        for (cnt = 0;;cnt++) { // 寻找可以整除dma_size的buffer大小
            if ((config->max_buffer_size - cnt) % (lcd_obj->dma_size * 2) == 0) {
                break;
            }
        }
        lcd_obj->buffer_size = config->max_buffer_size - cnt;
    } else {
        lcd_obj->dma_size = config->max_buffer_size / 2;
        lcd_obj->buffer_size = lcd_obj->dma_size * 2;
    }
    
    lcd_obj->half_buffer_size = lcd_obj->buffer_size / 2;

    lcd_obj->node_cnt = (lcd_obj->buffer_size) / lcd_obj->dma_size; // DMA节点个数
    lcd_obj->half_node_cnt = lcd_obj->node_cnt / 2;

    ESP_LOGI(TAG, "lcd_buffer_size: %d, lcd_dma_size: %d, lcd_dma_node_cnt: %d\n", lcd_obj->buffer_size, lcd_obj->dma_size, lcd_obj->node_cnt);

    lcd_obj->dma    = (lldesc_t *)heap_caps_malloc(lcd_obj->node_cnt * sizeof(lldesc_t), MALLOC_CAP_DMA);
    lcd_obj->buffer = (uint8_t *)heap_caps_malloc(lcd_obj->buffer_size * sizeof(uint8_t), MALLOC_CAP_DMA);
}

int lcd_init(lcd_config_t *config)
{
    lcd_obj = (lcd_obj_t *)heap_caps_calloc(1, sizeof(lcd_obj_t), MALLOC_CAP_DMA);
    if (!lcd_obj) {
        ESP_LOGI(TAG, "lcd object malloc error\n");
        return -1;
    }

    lcd_set_pin(config);
    lcd_config(config);
    lcd_dma_config(config);

    lcd_obj->event_queue = xQueueCreate(1, sizeof(int));
    
    
    lcd_obj->buffer_size = config->max_buffer_size;
    lcd_obj->bit_width = config->bit_width;
    lcd_obj->pin_dc = config->pin_dc;
    lcd_obj->pin_cs = config->pin_cs;
    lcd_obj->pin_rst = config->pin_rst;
    lcd_obj->pin_bk = config->pin_bk;
    lcd_set_cs(1);

    lcd_rst();//lcd_rst before LCD Init.
    lcd_delay_ms(100);
    lcd_ssd2805_config(config);
    lcd_config_lcm(config);
    // dcs_packet_trans();

    lcd_set_blk(0);
    ESP_LOGI(TAG, "lcd init ok\n");

    return 0;
}