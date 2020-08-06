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
}

static void lcd_delay_ms(uint32_t time)
{
    vTaskDelay(time / portTICK_RATE_MS);
}

static void lcd_write_cmd(uint16_t cmd)
{
    uint16_t val;
    if (lcd_obj->bit_width == 8) {
        val = (cmd >> 8) | (cmd << 8);
    } else {
        val = cmd;
    }
    lcd_obj->dc_state = 0;
    i2s_write_data(&val, 2);
}

static void lcd_write_reg(uint16_t cmd, uint16_t data)
{
    uint16_t val;
    lcd_write_cmd(cmd);

    if (lcd_obj->bit_width == 8) {
        val = (data >> 8) | (data << 8);
    } else {
        val = data;
    }
    lcd_obj->dc_state = 1;
    i2s_write_data(&val, 2);
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

static void lcd_nt35510_config(lcd_config_t *config)
{
    lcd_delay_ms(10);
    lcd_write_cmd(0x0100);
    lcd_write_cmd(0x0100);
    lcd_delay_ms(100);
    lcd_write_cmd(0x1200);
    lcd_write_reg(0xf000, 0x0055);
    lcd_write_reg(0xf001, 0x00aa);
    lcd_write_reg(0xf002, 0x0052);
    lcd_write_reg(0xf003, 0x0008);
    lcd_write_reg(0xf004, 0x0001);

    lcd_write_reg(0xbc01, 0x0086);
    lcd_write_reg(0xbc02, 0x006a);
    lcd_write_reg(0xbd01, 0x0086);
    lcd_write_reg(0xbd02, 0x006a);
    lcd_write_reg(0xbe01, 0x0067);

    lcd_write_reg(0xd100, 0x0000);
    lcd_write_reg(0xd101, 0x005d);
    lcd_write_reg(0xd102, 0x0000);
    lcd_write_reg(0xd103, 0x006b);
    lcd_write_reg(0xd104, 0x0000);
    lcd_write_reg(0xd105, 0x0084);
    lcd_write_reg(0xd106, 0x0000);
    lcd_write_reg(0xd107, 0x009c);
    lcd_write_reg(0xd108, 0x0000);
    lcd_write_reg(0xd109, 0x00b1);
    lcd_write_reg(0xd10a, 0x0000);
    lcd_write_reg(0xd10b, 0x00d9);
    lcd_write_reg(0xd10c, 0x0000);
    lcd_write_reg(0xd10d, 0x00fd);
    lcd_write_reg(0xd10e, 0x0001);
    lcd_write_reg(0xd10f, 0x0038);
    lcd_write_reg(0xd110, 0x0001);
    lcd_write_reg(0xd111, 0x0068);
    lcd_write_reg(0xd112, 0x0001);
    lcd_write_reg(0xd113, 0x00b9);
    lcd_write_reg(0xd114, 0x0001);
    lcd_write_reg(0xd115, 0x00fb);
    lcd_write_reg(0xd116, 0x0002);
    lcd_write_reg(0xd117, 0x0063);
    lcd_write_reg(0xd118, 0x0002);
    lcd_write_reg(0xd119, 0x00b9);
    lcd_write_reg(0xd11a, 0x0002);
    lcd_write_reg(0xd11b, 0x00bb);
    lcd_write_reg(0xd11c, 0x0003);
    lcd_write_reg(0xd11d, 0x0003);
    lcd_write_reg(0xd11e, 0x0003);
    lcd_write_reg(0xd11f, 0x0046);
    lcd_write_reg(0xd120, 0x0003);
    lcd_write_reg(0xd121, 0x0069);
    lcd_write_reg(0xd122, 0x0003);
    lcd_write_reg(0xd123, 0x008f);
    lcd_write_reg(0xd124, 0x0003);
    lcd_write_reg(0xd125, 0x00a4);
    lcd_write_reg(0xd126, 0x0003);
    lcd_write_reg(0xd127, 0x00b9);
    lcd_write_reg(0xd128, 0x0003);
    lcd_write_reg(0xd129, 0x00c7);
    lcd_write_reg(0xd12a, 0x0003);
    lcd_write_reg(0xd12b, 0x00c9);
    lcd_write_reg(0xd12c, 0x0003);
    lcd_write_reg(0xd12d, 0x00cb);
    lcd_write_reg(0xd12e, 0x0003);
    lcd_write_reg(0xd12f, 0x00cb);
    lcd_write_reg(0xd130, 0x0003);
    lcd_write_reg(0xd131, 0x00cb);
    lcd_write_reg(0xd132, 0x0003);
    lcd_write_reg(0xd133, 0x00cc);

    lcd_write_reg(0xd200, 0x0000);
    lcd_write_reg(0xd201, 0x005d);
    lcd_write_reg(0xd202, 0x0000);
    lcd_write_reg(0xd203, 0x006b);
    lcd_write_reg(0xd204, 0x0000);
    lcd_write_reg(0xd205, 0x0084);
    lcd_write_reg(0xd206, 0x0000);
    lcd_write_reg(0xd207, 0x009c);
    lcd_write_reg(0xd208, 0x0000);
    lcd_write_reg(0xd209, 0x00b1);
    lcd_write_reg(0xd20a, 0x0000);
    lcd_write_reg(0xd20b, 0x00d9);
    lcd_write_reg(0xd20c, 0x0000);
    lcd_write_reg(0xd20d, 0x00fd);
    lcd_write_reg(0xd20e, 0x0001);
    lcd_write_reg(0xd20f, 0x0038);
    lcd_write_reg(0xd210, 0x0001);
    lcd_write_reg(0xd211, 0x0068);
    lcd_write_reg(0xd212, 0x0001);
    lcd_write_reg(0xd213, 0x00b9);
    lcd_write_reg(0xd214, 0x0001);
    lcd_write_reg(0xd215, 0x00fb);
    lcd_write_reg(0xd216, 0x0002);
    lcd_write_reg(0xd217, 0x0063);
    lcd_write_reg(0xd218, 0x0002);
    lcd_write_reg(0xd219, 0x00b9);
    lcd_write_reg(0xd21a, 0x0002);
    lcd_write_reg(0xd21b, 0x00bb);
    lcd_write_reg(0xd21c, 0x0003);
    lcd_write_reg(0xd21d, 0x0003);
    lcd_write_reg(0xd21e, 0x0003);
    lcd_write_reg(0xd21f, 0x0046);
    lcd_write_reg(0xd220, 0x0003);
    lcd_write_reg(0xd221, 0x0069);
    lcd_write_reg(0xd222, 0x0003);
    lcd_write_reg(0xd223, 0x008f);
    lcd_write_reg(0xd224, 0x0003);
    lcd_write_reg(0xd225, 0x00a4);
    lcd_write_reg(0xd226, 0x0003);
    lcd_write_reg(0xd227, 0x00b9);
    lcd_write_reg(0xd228, 0x0003);
    lcd_write_reg(0xd229, 0x00c7);
    lcd_write_reg(0xd22a, 0x0003);
    lcd_write_reg(0xd22b, 0x00c9);
    lcd_write_reg(0xd22c, 0x0003);
    lcd_write_reg(0xd22d, 0x00cb);
    lcd_write_reg(0xd22e, 0x0003);
    lcd_write_reg(0xd22f, 0x00cb);
    lcd_write_reg(0xd230, 0x0003);
    lcd_write_reg(0xd231, 0x00cb);
    lcd_write_reg(0xd232, 0x0003);
    lcd_write_reg(0xd233, 0x00cc);

    lcd_write_reg(0xd300, 0x0000);
    lcd_write_reg(0xd301, 0x005d);
    lcd_write_reg(0xd302, 0x0000);
    lcd_write_reg(0xd303, 0x006b);
    lcd_write_reg(0xd304, 0x0000);
    lcd_write_reg(0xd305, 0x0084);
    lcd_write_reg(0xd306, 0x0000);
    lcd_write_reg(0xd307, 0x009c);
    lcd_write_reg(0xd308, 0x0000);
    lcd_write_reg(0xd309, 0x00b1);
    lcd_write_reg(0xd30a, 0x0000);
    lcd_write_reg(0xd30b, 0x00d9);
    lcd_write_reg(0xd30c, 0x0000);
    lcd_write_reg(0xd30d, 0x00fd);
    lcd_write_reg(0xd30e, 0x0001);
    lcd_write_reg(0xd30f, 0x0038);
    lcd_write_reg(0xd310, 0x0001);
    lcd_write_reg(0xd311, 0x0068);
    lcd_write_reg(0xd312, 0x0001);
    lcd_write_reg(0xd313, 0x00b9);
    lcd_write_reg(0xd314, 0x0001);
    lcd_write_reg(0xd315, 0x00fb);
    lcd_write_reg(0xd316, 0x0002);
    lcd_write_reg(0xd317, 0x0063);
    lcd_write_reg(0xd318, 0x0002);
    lcd_write_reg(0xd319, 0x00b9);
    lcd_write_reg(0xd31a, 0x0002);
    lcd_write_reg(0xd31b, 0x00bb);
    lcd_write_reg(0xd31c, 0x0003);
    lcd_write_reg(0xd31d, 0x0003);
    lcd_write_reg(0xd31e, 0x0003);
    lcd_write_reg(0xd31f, 0x0046);
    lcd_write_reg(0xd320, 0x0003);
    lcd_write_reg(0xd321, 0x0069);
    lcd_write_reg(0xd322, 0x0003);
    lcd_write_reg(0xd323, 0x008f);
    lcd_write_reg(0xd324, 0x0003);
    lcd_write_reg(0xd325, 0x00a4);
    lcd_write_reg(0xd326, 0x0003);
    lcd_write_reg(0xd327, 0x00b9);
    lcd_write_reg(0xd328, 0x0003);
    lcd_write_reg(0xd329, 0x00c7);
    lcd_write_reg(0xd32a, 0x0003);
    lcd_write_reg(0xd32b, 0x00c9);
    lcd_write_reg(0xd32c, 0x0003);
    lcd_write_reg(0xd32d, 0x00cb);
    lcd_write_reg(0xd32e, 0x0003);
    lcd_write_reg(0xd32f, 0x00cb);
    lcd_write_reg(0xd330, 0x0003);
    lcd_write_reg(0xd331, 0x00cb);
    lcd_write_reg(0xd332, 0x0003);
    lcd_write_reg(0xd333, 0x00cc);

    lcd_write_reg(0xd400, 0x0000);
    lcd_write_reg(0xd401, 0x005d);
    lcd_write_reg(0xd402, 0x0000);
    lcd_write_reg(0xd403, 0x006b);
    lcd_write_reg(0xd404, 0x0000);
    lcd_write_reg(0xd405, 0x0084);
    lcd_write_reg(0xd406, 0x0000);
    lcd_write_reg(0xd407, 0x009c);
    lcd_write_reg(0xd408, 0x0000);
    lcd_write_reg(0xd409, 0x00b1);
    lcd_write_reg(0xd40a, 0x0000);
    lcd_write_reg(0xd40b, 0x00d9);
    lcd_write_reg(0xd40c, 0x0000);
    lcd_write_reg(0xd40d, 0x00fd);
    lcd_write_reg(0xd40e, 0x0001);
    lcd_write_reg(0xd40f, 0x0038);
    lcd_write_reg(0xd410, 0x0001);
    lcd_write_reg(0xd411, 0x0068);
    lcd_write_reg(0xd412, 0x0001);
    lcd_write_reg(0xd413, 0x00b9);
    lcd_write_reg(0xd414, 0x0001);
    lcd_write_reg(0xd415, 0x00fb);
    lcd_write_reg(0xd416, 0x0002);
    lcd_write_reg(0xd417, 0x0063);
    lcd_write_reg(0xd418, 0x0002);
    lcd_write_reg(0xd419, 0x00b9);
    lcd_write_reg(0xd41a, 0x0002);
    lcd_write_reg(0xd41b, 0x00bb);
    lcd_write_reg(0xd41c, 0x0003);
    lcd_write_reg(0xd41d, 0x0003);
    lcd_write_reg(0xd41e, 0x0003);
    lcd_write_reg(0xd41f, 0x0046);
    lcd_write_reg(0xd420, 0x0003);
    lcd_write_reg(0xd421, 0x0069);
    lcd_write_reg(0xd422, 0x0003);
    lcd_write_reg(0xd423, 0x008f);
    lcd_write_reg(0xd424, 0x0003);
    lcd_write_reg(0xd425, 0x00a4);
    lcd_write_reg(0xd426, 0x0003);
    lcd_write_reg(0xd427, 0x00b9);
    lcd_write_reg(0xd428, 0x0003);
    lcd_write_reg(0xd429, 0x00c7);
    lcd_write_reg(0xd42a, 0x0003);
    lcd_write_reg(0xd42b, 0x00c9);
    lcd_write_reg(0xd42c, 0x0003);
    lcd_write_reg(0xd42d, 0x00cb);
    lcd_write_reg(0xd42e, 0x0003);
    lcd_write_reg(0xd42f, 0x00cb);
    lcd_write_reg(0xd430, 0x0003);
    lcd_write_reg(0xd431, 0x00cb);
    lcd_write_reg(0xd432, 0x0003);
    lcd_write_reg(0xd433, 0x00cc);

    lcd_write_reg(0xd500, 0x0000);
    lcd_write_reg(0xd501, 0x005d);
    lcd_write_reg(0xd502, 0x0000);
    lcd_write_reg(0xd503, 0x006b);
    lcd_write_reg(0xd504, 0x0000);
    lcd_write_reg(0xd505, 0x0084);
    lcd_write_reg(0xd506, 0x0000);
    lcd_write_reg(0xd507, 0x009c);
    lcd_write_reg(0xd508, 0x0000);
    lcd_write_reg(0xd509, 0x00b1);
    lcd_write_reg(0xd50a, 0x0000);
    lcd_write_reg(0xd50b, 0x00D9);
    lcd_write_reg(0xd50c, 0x0000);
    lcd_write_reg(0xd50d, 0x00fd);
    lcd_write_reg(0xd50e, 0x0001);
    lcd_write_reg(0xd50f, 0x0038);
    lcd_write_reg(0xd510, 0x0001);
    lcd_write_reg(0xd511, 0x0068);
    lcd_write_reg(0xd512, 0x0001);
    lcd_write_reg(0xd513, 0x00b9);
    lcd_write_reg(0xd514, 0x0001);
    lcd_write_reg(0xd515, 0x00fb);
    lcd_write_reg(0xd516, 0x0002);
    lcd_write_reg(0xd517, 0x0063);
    lcd_write_reg(0xd518, 0x0002);
    lcd_write_reg(0xd519, 0x00b9);
    lcd_write_reg(0xd51a, 0x0002);
    lcd_write_reg(0xd51b, 0x00bb);
    lcd_write_reg(0xd51c, 0x0003);
    lcd_write_reg(0xd51d, 0x0003);
    lcd_write_reg(0xd51e, 0x0003);
    lcd_write_reg(0xd51f, 0x0046);
    lcd_write_reg(0xd520, 0x0003);
    lcd_write_reg(0xd521, 0x0069);
    lcd_write_reg(0xd522, 0x0003);
    lcd_write_reg(0xd523, 0x008f);
    lcd_write_reg(0xd524, 0x0003);
    lcd_write_reg(0xd525, 0x00a4);
    lcd_write_reg(0xd526, 0x0003);
    lcd_write_reg(0xd527, 0x00b9);
    lcd_write_reg(0xd528, 0x0003);
    lcd_write_reg(0xd529, 0x00c7);
    lcd_write_reg(0xd52a, 0x0003);
    lcd_write_reg(0xd52b, 0x00c9);
    lcd_write_reg(0xd52c, 0x0003);
    lcd_write_reg(0xd52d, 0x00cb);
    lcd_write_reg(0xd52e, 0x0003);
    lcd_write_reg(0xd52f, 0x00cb);
    lcd_write_reg(0xd530, 0x0003);
    lcd_write_reg(0xd531, 0x00cb);
    lcd_write_reg(0xd532, 0x0003);
    lcd_write_reg(0xd533, 0x00cc);

    lcd_write_reg(0xd600, 0x0000);
    lcd_write_reg(0xd601, 0x005d);
    lcd_write_reg(0xd602, 0x0000);
    lcd_write_reg(0xd603, 0x006b);
    lcd_write_reg(0xd604, 0x0000);
    lcd_write_reg(0xd605, 0x0084);
    lcd_write_reg(0xd606, 0x0000);
    lcd_write_reg(0xd607, 0x009c);
    lcd_write_reg(0xd608, 0x0000);
    lcd_write_reg(0xd609, 0x00b1);
    lcd_write_reg(0xd60a, 0x0000);
    lcd_write_reg(0xd60b, 0x00d9);
    lcd_write_reg(0xd60c, 0x0000);
    lcd_write_reg(0xd60d, 0x00fd);
    lcd_write_reg(0xd60e, 0x0001);
    lcd_write_reg(0xd60f, 0x0038);
    lcd_write_reg(0xd610, 0x0001);
    lcd_write_reg(0xd611, 0x0068);
    lcd_write_reg(0xd612, 0x0001);
    lcd_write_reg(0xd613, 0x00b9);
    lcd_write_reg(0xd614, 0x0001);
    lcd_write_reg(0xd615, 0x00fb);
    lcd_write_reg(0xd616, 0x0002);
    lcd_write_reg(0xd617, 0x0063);
    lcd_write_reg(0xd618, 0x0002);
    lcd_write_reg(0xd619, 0x00b9);
    lcd_write_reg(0xd61a, 0x0002);
    lcd_write_reg(0xd61b, 0x00bb);
    lcd_write_reg(0xd61c, 0x0003);
    lcd_write_reg(0xd61d, 0x0003);
    lcd_write_reg(0xd61e, 0x0003);
    lcd_write_reg(0xd61f, 0x0046);
    lcd_write_reg(0xd620, 0x0003);
    lcd_write_reg(0xd621, 0x0069);
    lcd_write_reg(0xd622, 0x0003);
    lcd_write_reg(0xd623, 0x008f);
    lcd_write_reg(0xd624, 0x0003);
    lcd_write_reg(0xd625, 0x00a4);
    lcd_write_reg(0xd626, 0x0003);
    lcd_write_reg(0xd627, 0x00b9);
    lcd_write_reg(0xd628, 0x0003);
    lcd_write_reg(0xd629, 0x00c7);
    lcd_write_reg(0xd62a, 0x0003);
    lcd_write_reg(0xd62b, 0x00c9);
    lcd_write_reg(0xd62c, 0x0003);
    lcd_write_reg(0xd62d, 0x00cb);
    lcd_write_reg(0xd62e, 0x0003);
    lcd_write_reg(0xd62f, 0x00cb);
    lcd_write_reg(0xd630, 0x0003);
    lcd_write_reg(0xd631, 0x00cb);
    lcd_write_reg(0xd632, 0x0003);
    lcd_write_reg(0xd633, 0x00cc);

    lcd_write_reg(0xba00, 0x0024);
    lcd_write_reg(0xba01, 0x0024);
    lcd_write_reg(0xba02, 0x0024);

    lcd_write_reg(0xb900, 0x0024);
    lcd_write_reg(0xb901, 0x0024);
    lcd_write_reg(0xb902, 0x0024);

    lcd_write_reg(0xf000, 0x0055);
    lcd_write_reg(0xf001, 0x00aa);
    lcd_write_reg(0xf002, 0x0052);
    lcd_write_reg(0xf003, 0x0008);
    lcd_write_reg(0xf004, 0x0000);

    lcd_write_reg(0xb100, 0x00cc);
    lcd_write_reg(0xB500, 0x0050);

    lcd_write_reg(0xbc00, 0x0005);
    lcd_write_reg(0xbc01, 0x0005);
    lcd_write_reg(0xbc02, 0x0005);

    lcd_write_reg(0xb800, 0x0001);
    lcd_write_reg(0xb801, 0x0003);
    lcd_write_reg(0xb802, 0x0003);
    lcd_write_reg(0xb803, 0x0003);

    lcd_write_reg(0xbd02, 0x0007);
    lcd_write_reg(0xbd03, 0x0031);
    lcd_write_reg(0xbe02, 0x0007);
    lcd_write_reg(0xbe03, 0x0031);
    lcd_write_reg(0xbf02, 0x0007);
    lcd_write_reg(0xbf03, 0x0031);

    lcd_write_reg(0xff00, 0x00aa);
    lcd_write_reg(0xff01, 0x0055);
    lcd_write_reg(0xff02, 0x0025);
    lcd_write_reg(0xff03, 0x0001);

    lcd_write_reg(0xf304, 0x0011);
    lcd_write_reg(0xf306, 0x0010);
    lcd_write_reg(0xf308, 0x0000);

    lcd_write_reg(0x3500, 0x0000);
    lcd_write_reg(0x3600, 0x0060);
    
    lcd_write_reg(0x3A00, 0x0005);
    //Display On
    lcd_write_cmd(0x2900);
    // Out sleep
    lcd_write_cmd(0x1100);
    // Write continue
    lcd_write_cmd(0x2C00);
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
    lcd_nt35510_config(config);

    lcd_set_blk(0);
    ESP_LOGI(TAG, "lcd init ok\n");

    return 0;
}

void lcd_set_index(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end)
{
    lcd_write_reg(0x2A00, (x_start >> 8));
    lcd_write_reg(0x2A01, (x_start & 0xff));
    lcd_write_reg(0x2A02, (x_end >> 8));
    lcd_write_reg(0x2A03, (x_end & 0xff));

    lcd_write_reg(0x2B00, (y_start >> 8));
    lcd_write_reg(0x2B01, (y_start & 0xff));
    lcd_write_reg(0x2B02, (y_end >> 8));
    lcd_write_reg(0x2B03, (y_end & 0xff));

    lcd_write_cmd(0x2C00);
}