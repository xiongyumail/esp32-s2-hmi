#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "nt35510.h"

static const char *TAG = "nt35510";

typedef struct {
    nt35510_config_t config;
    nt35510_write_callback_t write_cb;
} nt35510_obj_t;

static nt35510_obj_t *nt35510_obj = NULL;

static void inline nt35510_set_level(int8_t io_num, uint8_t state, bool invert)
{
    if (io_num < 0) {
        return;
    }
    gpio_set_level(io_num, invert ? !state : state);
}

static void nt35510_delay_ms(uint32_t time)
{
    vTaskDelay(time / portTICK_RATE_MS);
}

static void nt35510_write_cmd(uint16_t cmd)
{
    uint16_t val;
    if (nt35510_obj->config.width == 8) {
        val = (cmd >> 8) | (cmd << 8);
    } else {
        val = cmd;
    }
    nt35510_set_level(nt35510_obj->config.pin.cs, 0, nt35510_obj->config.invert.cs);
    nt35510_set_level(nt35510_obj->config.pin.dc, 0, nt35510_obj->config.invert.dc);
    nt35510_obj->write_cb(&val, 2);
    nt35510_set_level(nt35510_obj->config.pin.cs, 1, nt35510_obj->config.invert.cs);
}

static void nt35510_write_reg(uint16_t cmd, uint16_t data)
{
    uint16_t val;
    nt35510_write_cmd(cmd);

    if (nt35510_obj->config.width == 8) {
        val = (data >> 8) | (data << 8);
    } else {
        val = data;
    }
    nt35510_set_level(nt35510_obj->config.pin.cs, 0, nt35510_obj->config.invert.cs);
    nt35510_set_level(nt35510_obj->config.pin.dc, 1, nt35510_obj->config.invert.dc);
    nt35510_obj->write_cb(&val, 2);
    nt35510_set_level(nt35510_obj->config.pin.cs, 1, nt35510_obj->config.invert.cs);
}

void nt35510_write_data(uint8_t *data, size_t len)
{
    if (len <= 0) {
        return;
    }
    nt35510_set_level(nt35510_obj->config.pin.cs, 0, nt35510_obj->config.invert.cs);
    nt35510_set_level(nt35510_obj->config.pin.dc, 1, nt35510_obj->config.invert.dc);
    nt35510_obj->write_cb(data, len);
    nt35510_set_level(nt35510_obj->config.pin.cs, 1, nt35510_obj->config.invert.cs);
}

static void nt35510_rst()
{
    nt35510_set_level(nt35510_obj->config.pin.rst, 0, nt35510_obj->config.invert.rst);
    nt35510_delay_ms(100);
    nt35510_set_level(nt35510_obj->config.pin.rst, 1, nt35510_obj->config.invert.rst);
    nt35510_delay_ms(100);
}

static void nt35510_config(nt35510_config_t *config)
{
    nt35510_delay_ms(10);
    nt35510_write_cmd(0x0100);
    nt35510_write_cmd(0x0100);
    nt35510_delay_ms(100);
    nt35510_write_cmd(0x1200);
    nt35510_write_reg(0xf000, 0x0055);
    nt35510_write_reg(0xf001, 0x00aa);
    nt35510_write_reg(0xf002, 0x0052);
    nt35510_write_reg(0xf003, 0x0008);
    nt35510_write_reg(0xf004, 0x0001);

    nt35510_write_reg(0xbc01, 0x0086);
    nt35510_write_reg(0xbc02, 0x006a);
    nt35510_write_reg(0xbd01, 0x0086);
    nt35510_write_reg(0xbd02, 0x006a);
    nt35510_write_reg(0xbe01, 0x0067);

    nt35510_write_reg(0xd100, 0x0000);
    nt35510_write_reg(0xd101, 0x005d);
    nt35510_write_reg(0xd102, 0x0000);
    nt35510_write_reg(0xd103, 0x006b);
    nt35510_write_reg(0xd104, 0x0000);
    nt35510_write_reg(0xd105, 0x0084);
    nt35510_write_reg(0xd106, 0x0000);
    nt35510_write_reg(0xd107, 0x009c);
    nt35510_write_reg(0xd108, 0x0000);
    nt35510_write_reg(0xd109, 0x00b1);
    nt35510_write_reg(0xd10a, 0x0000);
    nt35510_write_reg(0xd10b, 0x00d9);
    nt35510_write_reg(0xd10c, 0x0000);
    nt35510_write_reg(0xd10d, 0x00fd);
    nt35510_write_reg(0xd10e, 0x0001);
    nt35510_write_reg(0xd10f, 0x0038);
    nt35510_write_reg(0xd110, 0x0001);
    nt35510_write_reg(0xd111, 0x0068);
    nt35510_write_reg(0xd112, 0x0001);
    nt35510_write_reg(0xd113, 0x00b9);
    nt35510_write_reg(0xd114, 0x0001);
    nt35510_write_reg(0xd115, 0x00fb);
    nt35510_write_reg(0xd116, 0x0002);
    nt35510_write_reg(0xd117, 0x0063);
    nt35510_write_reg(0xd118, 0x0002);
    nt35510_write_reg(0xd119, 0x00b9);
    nt35510_write_reg(0xd11a, 0x0002);
    nt35510_write_reg(0xd11b, 0x00bb);
    nt35510_write_reg(0xd11c, 0x0003);
    nt35510_write_reg(0xd11d, 0x0003);
    nt35510_write_reg(0xd11e, 0x0003);
    nt35510_write_reg(0xd11f, 0x0046);
    nt35510_write_reg(0xd120, 0x0003);
    nt35510_write_reg(0xd121, 0x0069);
    nt35510_write_reg(0xd122, 0x0003);
    nt35510_write_reg(0xd123, 0x008f);
    nt35510_write_reg(0xd124, 0x0003);
    nt35510_write_reg(0xd125, 0x00a4);
    nt35510_write_reg(0xd126, 0x0003);
    nt35510_write_reg(0xd127, 0x00b9);
    nt35510_write_reg(0xd128, 0x0003);
    nt35510_write_reg(0xd129, 0x00c7);
    nt35510_write_reg(0xd12a, 0x0003);
    nt35510_write_reg(0xd12b, 0x00c9);
    nt35510_write_reg(0xd12c, 0x0003);
    nt35510_write_reg(0xd12d, 0x00cb);
    nt35510_write_reg(0xd12e, 0x0003);
    nt35510_write_reg(0xd12f, 0x00cb);
    nt35510_write_reg(0xd130, 0x0003);
    nt35510_write_reg(0xd131, 0x00cb);
    nt35510_write_reg(0xd132, 0x0003);
    nt35510_write_reg(0xd133, 0x00cc);

    nt35510_write_reg(0xd200, 0x0000);
    nt35510_write_reg(0xd201, 0x005d);
    nt35510_write_reg(0xd202, 0x0000);
    nt35510_write_reg(0xd203, 0x006b);
    nt35510_write_reg(0xd204, 0x0000);
    nt35510_write_reg(0xd205, 0x0084);
    nt35510_write_reg(0xd206, 0x0000);
    nt35510_write_reg(0xd207, 0x009c);
    nt35510_write_reg(0xd208, 0x0000);
    nt35510_write_reg(0xd209, 0x00b1);
    nt35510_write_reg(0xd20a, 0x0000);
    nt35510_write_reg(0xd20b, 0x00d9);
    nt35510_write_reg(0xd20c, 0x0000);
    nt35510_write_reg(0xd20d, 0x00fd);
    nt35510_write_reg(0xd20e, 0x0001);
    nt35510_write_reg(0xd20f, 0x0038);
    nt35510_write_reg(0xd210, 0x0001);
    nt35510_write_reg(0xd211, 0x0068);
    nt35510_write_reg(0xd212, 0x0001);
    nt35510_write_reg(0xd213, 0x00b9);
    nt35510_write_reg(0xd214, 0x0001);
    nt35510_write_reg(0xd215, 0x00fb);
    nt35510_write_reg(0xd216, 0x0002);
    nt35510_write_reg(0xd217, 0x0063);
    nt35510_write_reg(0xd218, 0x0002);
    nt35510_write_reg(0xd219, 0x00b9);
    nt35510_write_reg(0xd21a, 0x0002);
    nt35510_write_reg(0xd21b, 0x00bb);
    nt35510_write_reg(0xd21c, 0x0003);
    nt35510_write_reg(0xd21d, 0x0003);
    nt35510_write_reg(0xd21e, 0x0003);
    nt35510_write_reg(0xd21f, 0x0046);
    nt35510_write_reg(0xd220, 0x0003);
    nt35510_write_reg(0xd221, 0x0069);
    nt35510_write_reg(0xd222, 0x0003);
    nt35510_write_reg(0xd223, 0x008f);
    nt35510_write_reg(0xd224, 0x0003);
    nt35510_write_reg(0xd225, 0x00a4);
    nt35510_write_reg(0xd226, 0x0003);
    nt35510_write_reg(0xd227, 0x00b9);
    nt35510_write_reg(0xd228, 0x0003);
    nt35510_write_reg(0xd229, 0x00c7);
    nt35510_write_reg(0xd22a, 0x0003);
    nt35510_write_reg(0xd22b, 0x00c9);
    nt35510_write_reg(0xd22c, 0x0003);
    nt35510_write_reg(0xd22d, 0x00cb);
    nt35510_write_reg(0xd22e, 0x0003);
    nt35510_write_reg(0xd22f, 0x00cb);
    nt35510_write_reg(0xd230, 0x0003);
    nt35510_write_reg(0xd231, 0x00cb);
    nt35510_write_reg(0xd232, 0x0003);
    nt35510_write_reg(0xd233, 0x00cc);

    nt35510_write_reg(0xd300, 0x0000);
    nt35510_write_reg(0xd301, 0x005d);
    nt35510_write_reg(0xd302, 0x0000);
    nt35510_write_reg(0xd303, 0x006b);
    nt35510_write_reg(0xd304, 0x0000);
    nt35510_write_reg(0xd305, 0x0084);
    nt35510_write_reg(0xd306, 0x0000);
    nt35510_write_reg(0xd307, 0x009c);
    nt35510_write_reg(0xd308, 0x0000);
    nt35510_write_reg(0xd309, 0x00b1);
    nt35510_write_reg(0xd30a, 0x0000);
    nt35510_write_reg(0xd30b, 0x00d9);
    nt35510_write_reg(0xd30c, 0x0000);
    nt35510_write_reg(0xd30d, 0x00fd);
    nt35510_write_reg(0xd30e, 0x0001);
    nt35510_write_reg(0xd30f, 0x0038);
    nt35510_write_reg(0xd310, 0x0001);
    nt35510_write_reg(0xd311, 0x0068);
    nt35510_write_reg(0xd312, 0x0001);
    nt35510_write_reg(0xd313, 0x00b9);
    nt35510_write_reg(0xd314, 0x0001);
    nt35510_write_reg(0xd315, 0x00fb);
    nt35510_write_reg(0xd316, 0x0002);
    nt35510_write_reg(0xd317, 0x0063);
    nt35510_write_reg(0xd318, 0x0002);
    nt35510_write_reg(0xd319, 0x00b9);
    nt35510_write_reg(0xd31a, 0x0002);
    nt35510_write_reg(0xd31b, 0x00bb);
    nt35510_write_reg(0xd31c, 0x0003);
    nt35510_write_reg(0xd31d, 0x0003);
    nt35510_write_reg(0xd31e, 0x0003);
    nt35510_write_reg(0xd31f, 0x0046);
    nt35510_write_reg(0xd320, 0x0003);
    nt35510_write_reg(0xd321, 0x0069);
    nt35510_write_reg(0xd322, 0x0003);
    nt35510_write_reg(0xd323, 0x008f);
    nt35510_write_reg(0xd324, 0x0003);
    nt35510_write_reg(0xd325, 0x00a4);
    nt35510_write_reg(0xd326, 0x0003);
    nt35510_write_reg(0xd327, 0x00b9);
    nt35510_write_reg(0xd328, 0x0003);
    nt35510_write_reg(0xd329, 0x00c7);
    nt35510_write_reg(0xd32a, 0x0003);
    nt35510_write_reg(0xd32b, 0x00c9);
    nt35510_write_reg(0xd32c, 0x0003);
    nt35510_write_reg(0xd32d, 0x00cb);
    nt35510_write_reg(0xd32e, 0x0003);
    nt35510_write_reg(0xd32f, 0x00cb);
    nt35510_write_reg(0xd330, 0x0003);
    nt35510_write_reg(0xd331, 0x00cb);
    nt35510_write_reg(0xd332, 0x0003);
    nt35510_write_reg(0xd333, 0x00cc);

    nt35510_write_reg(0xd400, 0x0000);
    nt35510_write_reg(0xd401, 0x005d);
    nt35510_write_reg(0xd402, 0x0000);
    nt35510_write_reg(0xd403, 0x006b);
    nt35510_write_reg(0xd404, 0x0000);
    nt35510_write_reg(0xd405, 0x0084);
    nt35510_write_reg(0xd406, 0x0000);
    nt35510_write_reg(0xd407, 0x009c);
    nt35510_write_reg(0xd408, 0x0000);
    nt35510_write_reg(0xd409, 0x00b1);
    nt35510_write_reg(0xd40a, 0x0000);
    nt35510_write_reg(0xd40b, 0x00d9);
    nt35510_write_reg(0xd40c, 0x0000);
    nt35510_write_reg(0xd40d, 0x00fd);
    nt35510_write_reg(0xd40e, 0x0001);
    nt35510_write_reg(0xd40f, 0x0038);
    nt35510_write_reg(0xd410, 0x0001);
    nt35510_write_reg(0xd411, 0x0068);
    nt35510_write_reg(0xd412, 0x0001);
    nt35510_write_reg(0xd413, 0x00b9);
    nt35510_write_reg(0xd414, 0x0001);
    nt35510_write_reg(0xd415, 0x00fb);
    nt35510_write_reg(0xd416, 0x0002);
    nt35510_write_reg(0xd417, 0x0063);
    nt35510_write_reg(0xd418, 0x0002);
    nt35510_write_reg(0xd419, 0x00b9);
    nt35510_write_reg(0xd41a, 0x0002);
    nt35510_write_reg(0xd41b, 0x00bb);
    nt35510_write_reg(0xd41c, 0x0003);
    nt35510_write_reg(0xd41d, 0x0003);
    nt35510_write_reg(0xd41e, 0x0003);
    nt35510_write_reg(0xd41f, 0x0046);
    nt35510_write_reg(0xd420, 0x0003);
    nt35510_write_reg(0xd421, 0x0069);
    nt35510_write_reg(0xd422, 0x0003);
    nt35510_write_reg(0xd423, 0x008f);
    nt35510_write_reg(0xd424, 0x0003);
    nt35510_write_reg(0xd425, 0x00a4);
    nt35510_write_reg(0xd426, 0x0003);
    nt35510_write_reg(0xd427, 0x00b9);
    nt35510_write_reg(0xd428, 0x0003);
    nt35510_write_reg(0xd429, 0x00c7);
    nt35510_write_reg(0xd42a, 0x0003);
    nt35510_write_reg(0xd42b, 0x00c9);
    nt35510_write_reg(0xd42c, 0x0003);
    nt35510_write_reg(0xd42d, 0x00cb);
    nt35510_write_reg(0xd42e, 0x0003);
    nt35510_write_reg(0xd42f, 0x00cb);
    nt35510_write_reg(0xd430, 0x0003);
    nt35510_write_reg(0xd431, 0x00cb);
    nt35510_write_reg(0xd432, 0x0003);
    nt35510_write_reg(0xd433, 0x00cc);

    nt35510_write_reg(0xd500, 0x0000);
    nt35510_write_reg(0xd501, 0x005d);
    nt35510_write_reg(0xd502, 0x0000);
    nt35510_write_reg(0xd503, 0x006b);
    nt35510_write_reg(0xd504, 0x0000);
    nt35510_write_reg(0xd505, 0x0084);
    nt35510_write_reg(0xd506, 0x0000);
    nt35510_write_reg(0xd507, 0x009c);
    nt35510_write_reg(0xd508, 0x0000);
    nt35510_write_reg(0xd509, 0x00b1);
    nt35510_write_reg(0xd50a, 0x0000);
    nt35510_write_reg(0xd50b, 0x00D9);
    nt35510_write_reg(0xd50c, 0x0000);
    nt35510_write_reg(0xd50d, 0x00fd);
    nt35510_write_reg(0xd50e, 0x0001);
    nt35510_write_reg(0xd50f, 0x0038);
    nt35510_write_reg(0xd510, 0x0001);
    nt35510_write_reg(0xd511, 0x0068);
    nt35510_write_reg(0xd512, 0x0001);
    nt35510_write_reg(0xd513, 0x00b9);
    nt35510_write_reg(0xd514, 0x0001);
    nt35510_write_reg(0xd515, 0x00fb);
    nt35510_write_reg(0xd516, 0x0002);
    nt35510_write_reg(0xd517, 0x0063);
    nt35510_write_reg(0xd518, 0x0002);
    nt35510_write_reg(0xd519, 0x00b9);
    nt35510_write_reg(0xd51a, 0x0002);
    nt35510_write_reg(0xd51b, 0x00bb);
    nt35510_write_reg(0xd51c, 0x0003);
    nt35510_write_reg(0xd51d, 0x0003);
    nt35510_write_reg(0xd51e, 0x0003);
    nt35510_write_reg(0xd51f, 0x0046);
    nt35510_write_reg(0xd520, 0x0003);
    nt35510_write_reg(0xd521, 0x0069);
    nt35510_write_reg(0xd522, 0x0003);
    nt35510_write_reg(0xd523, 0x008f);
    nt35510_write_reg(0xd524, 0x0003);
    nt35510_write_reg(0xd525, 0x00a4);
    nt35510_write_reg(0xd526, 0x0003);
    nt35510_write_reg(0xd527, 0x00b9);
    nt35510_write_reg(0xd528, 0x0003);
    nt35510_write_reg(0xd529, 0x00c7);
    nt35510_write_reg(0xd52a, 0x0003);
    nt35510_write_reg(0xd52b, 0x00c9);
    nt35510_write_reg(0xd52c, 0x0003);
    nt35510_write_reg(0xd52d, 0x00cb);
    nt35510_write_reg(0xd52e, 0x0003);
    nt35510_write_reg(0xd52f, 0x00cb);
    nt35510_write_reg(0xd530, 0x0003);
    nt35510_write_reg(0xd531, 0x00cb);
    nt35510_write_reg(0xd532, 0x0003);
    nt35510_write_reg(0xd533, 0x00cc);

    nt35510_write_reg(0xd600, 0x0000);
    nt35510_write_reg(0xd601, 0x005d);
    nt35510_write_reg(0xd602, 0x0000);
    nt35510_write_reg(0xd603, 0x006b);
    nt35510_write_reg(0xd604, 0x0000);
    nt35510_write_reg(0xd605, 0x0084);
    nt35510_write_reg(0xd606, 0x0000);
    nt35510_write_reg(0xd607, 0x009c);
    nt35510_write_reg(0xd608, 0x0000);
    nt35510_write_reg(0xd609, 0x00b1);
    nt35510_write_reg(0xd60a, 0x0000);
    nt35510_write_reg(0xd60b, 0x00d9);
    nt35510_write_reg(0xd60c, 0x0000);
    nt35510_write_reg(0xd60d, 0x00fd);
    nt35510_write_reg(0xd60e, 0x0001);
    nt35510_write_reg(0xd60f, 0x0038);
    nt35510_write_reg(0xd610, 0x0001);
    nt35510_write_reg(0xd611, 0x0068);
    nt35510_write_reg(0xd612, 0x0001);
    nt35510_write_reg(0xd613, 0x00b9);
    nt35510_write_reg(0xd614, 0x0001);
    nt35510_write_reg(0xd615, 0x00fb);
    nt35510_write_reg(0xd616, 0x0002);
    nt35510_write_reg(0xd617, 0x0063);
    nt35510_write_reg(0xd618, 0x0002);
    nt35510_write_reg(0xd619, 0x00b9);
    nt35510_write_reg(0xd61a, 0x0002);
    nt35510_write_reg(0xd61b, 0x00bb);
    nt35510_write_reg(0xd61c, 0x0003);
    nt35510_write_reg(0xd61d, 0x0003);
    nt35510_write_reg(0xd61e, 0x0003);
    nt35510_write_reg(0xd61f, 0x0046);
    nt35510_write_reg(0xd620, 0x0003);
    nt35510_write_reg(0xd621, 0x0069);
    nt35510_write_reg(0xd622, 0x0003);
    nt35510_write_reg(0xd623, 0x008f);
    nt35510_write_reg(0xd624, 0x0003);
    nt35510_write_reg(0xd625, 0x00a4);
    nt35510_write_reg(0xd626, 0x0003);
    nt35510_write_reg(0xd627, 0x00b9);
    nt35510_write_reg(0xd628, 0x0003);
    nt35510_write_reg(0xd629, 0x00c7);
    nt35510_write_reg(0xd62a, 0x0003);
    nt35510_write_reg(0xd62b, 0x00c9);
    nt35510_write_reg(0xd62c, 0x0003);
    nt35510_write_reg(0xd62d, 0x00cb);
    nt35510_write_reg(0xd62e, 0x0003);
    nt35510_write_reg(0xd62f, 0x00cb);
    nt35510_write_reg(0xd630, 0x0003);
    nt35510_write_reg(0xd631, 0x00cb);
    nt35510_write_reg(0xd632, 0x0003);
    nt35510_write_reg(0xd633, 0x00cc);

    nt35510_write_reg(0xba00, 0x0024);
    nt35510_write_reg(0xba01, 0x0024);
    nt35510_write_reg(0xba02, 0x0024);

    nt35510_write_reg(0xb900, 0x0024);
    nt35510_write_reg(0xb901, 0x0024);
    nt35510_write_reg(0xb902, 0x0024);

    nt35510_write_reg(0xf000, 0x0055);
    nt35510_write_reg(0xf001, 0x00aa);
    nt35510_write_reg(0xf002, 0x0052);
    nt35510_write_reg(0xf003, 0x0008);
    nt35510_write_reg(0xf004, 0x0000);

    nt35510_write_reg(0xb100, 0x00cc);
    nt35510_write_reg(0xB500, 0x0050);

    nt35510_write_reg(0xbc00, 0x0005);
    nt35510_write_reg(0xbc01, 0x0005);
    nt35510_write_reg(0xbc02, 0x0005);

    nt35510_write_reg(0xb800, 0x0001);
    nt35510_write_reg(0xb801, 0x0003);
    nt35510_write_reg(0xb802, 0x0003);
    nt35510_write_reg(0xb803, 0x0003);

    nt35510_write_reg(0xbd02, 0x0007);
    nt35510_write_reg(0xbd03, 0x0031);
    nt35510_write_reg(0xbe02, 0x0007);
    nt35510_write_reg(0xbe03, 0x0031);
    nt35510_write_reg(0xbf02, 0x0007);
    nt35510_write_reg(0xbf03, 0x0031);

    nt35510_write_reg(0xff00, 0x00aa);
    nt35510_write_reg(0xff01, 0x0055);
    nt35510_write_reg(0xff02, 0x0025);
    nt35510_write_reg(0xff03, 0x0001);

    nt35510_write_reg(0xf304, 0x0011);
    nt35510_write_reg(0xf306, 0x0010);
    nt35510_write_reg(0xf308, 0x0000);

    nt35510_write_reg(0x3500, 0x0000);
    nt35510_write_reg(0x3600, 0x0060);
    
    nt35510_write_reg(0x3A00, 0x0005);
    //Display On
    nt35510_write_cmd(0x2900);
    // Out sleep
    nt35510_write_cmd(0x1100);
    // Write continue
    nt35510_write_cmd(0x2C00);
}

void nt35510_set_index(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end)
{
    nt35510_write_reg(0x2A00, (x_start >> 8));
    nt35510_write_reg(0x2A01, (x_start & 0xff));
    nt35510_write_reg(0x2A02, (x_end >> 8));
    nt35510_write_reg(0x2A03, (x_end & 0xff));

    nt35510_write_reg(0x2B00, (y_start >> 8));
    nt35510_write_reg(0x2B01, (y_start & 0xff));
    nt35510_write_reg(0x2B02, (y_end >> 8));
    nt35510_write_reg(0x2B03, (y_end & 0xff));

    nt35510_write_cmd(0x2C00);
}


void nt35510_deinit(void)
{
    free(nt35510_obj);
}

int nt35510_init(nt35510_config_t *config)
{
    nt35510_obj = (nt35510_obj_t *)heap_caps_calloc(1, sizeof(nt35510_obj_t), MALLOC_CAP_DEFAULT);
    if (!nt35510_obj) {
        ESP_LOGE(TAG, "nt35510 object malloc error\n");
        return -1;
    }

    memcpy(&nt35510_obj->config, config, sizeof(nt35510_config_t));
    nt35510_obj->write_cb = config->write_cb;
    if (nt35510_obj->write_cb == NULL) {
        ESP_LOGE(TAG, "nt35510 write_callback NULL\n");
        nt35510_deinit();
        return -1;
    }

    //Initialize non-matrix GPIOs
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask  = (config->pin.dc < 0) ? 0ULL : (1ULL << config->pin.dc);
    io_conf.pin_bit_mask |= (config->pin.rd < 0) ? 0ULL : (1ULL << config->pin.rd);
    io_conf.pin_bit_mask |= (config->pin.rst < 0) ? 0ULL : (1ULL << config->pin.rst);
    io_conf.pin_bit_mask |= (config->pin.bk < 0) ? 0ULL : (1ULL << config->pin.bk);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    nt35510_set_level(nt35510_obj->config.pin.rd, 1, nt35510_obj->config.invert.rd);
    nt35510_set_level(nt35510_obj->config.pin.cs, 0, nt35510_obj->config.invert.cs);
    nt35510_rst();//nt35510_rst before LCD Init.
    nt35510_delay_ms(100);
    nt35510_config(config);
    if (nt35510_obj->config.width == 8 && (nt35510_obj->config.pin.rst == -1)) { // 当没有外部复位和位宽为8位时，需要配置两次寄存器
        nt35510_config(config);
    }
    nt35510_set_level(nt35510_obj->config.pin.bk, 0, nt35510_obj->config.invert.bk);
    
    return 0;
}
