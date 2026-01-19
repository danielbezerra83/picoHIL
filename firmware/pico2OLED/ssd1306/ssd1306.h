#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <string.h>
#include "ssd1306_fonts.h"

#define SSD1306_ADDR 0x3C   // Endereço I2C padrão
#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64

typedef struct {
    i2c_inst_t *i2c;
    uint8_t addr;
    uint8_t buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];
} ssd1306_t;

typedef struct {
    const uint8_t *data;   // ponteiro para tabela de caracteres
    uint8_t width;         // largura em pixels
    uint8_t height;        // altura em pixels
    uint8_t spacing;       // espaço entre caracteres
} font_t;


//extern static void ssd1306_send_cmd(ssd1306_t *disp, uint8_t cmd);
extern void ssd1306_init(ssd1306_t *disp, i2c_inst_t *i2c, uint8_t addr);
extern void ssd1306_show(ssd1306_t *disp);
extern void ssd1306_clear(ssd1306_t *disp);
extern void ssd1306_showall(ssd1306_t *disp);
extern void ssd1306_draw_string_font(ssd1306_t *disp, int x, int y, const char *str, const font_t *font);
