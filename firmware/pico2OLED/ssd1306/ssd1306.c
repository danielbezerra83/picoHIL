#include "ssd1306.h"

// Função auxiliar para enviar comando
static void ssd1306_send_cmd(ssd1306_t *disp, uint8_t cmd) {
    uint8_t buf[2] = {0x00, cmd}; // 0x00 = próximo byte é comando
    i2c_write_blocking(disp->i2c, disp->addr, buf, 2, false);
}

// Função auxiliar para enviar dados
static void ssd1306_send_data(ssd1306_t *disp, uint8_t *data, size_t len) {
    uint8_t buf[len + 1];
    buf[0] = 0x40; // 0x40 = próximo byte(s) são dados
    memcpy(&buf[1], data, len);
    i2c_write_blocking(disp->i2c, disp->addr, buf, len + 1, false);
}

// Inicialização básica
void ssd1306_init(ssd1306_t *disp, i2c_inst_t *i2c, uint8_t addr) {
    disp->i2c = i2c;
    disp->addr = addr;
    memset(disp->buffer, 0, sizeof(disp->buffer));

    // Sequência mínima de init
    ssd1306_send_cmd(disp, 0xAE); // Display OFF
    ssd1306_send_cmd(disp, 0xA8); ssd1306_send_cmd(disp, 0x3F); // Multiplex ratio
    ssd1306_send_cmd(disp, 0xD3); ssd1306_send_cmd(disp, 0x00); // Display offset
    ssd1306_send_cmd(disp, 0x40); // Start line = 0
    ssd1306_send_cmd(disp, 0xA1); // Segment remap
    ssd1306_send_cmd(disp, 0xC8); // COM scan direction
    ssd1306_send_cmd(disp, 0xDA); ssd1306_send_cmd(disp, 0x12); // COM pins
    ssd1306_send_cmd(disp, 0x81); ssd1306_send_cmd(disp, 0x7F); // Contrast
    ssd1306_send_cmd(disp, 0xA4); // Resume RAM content
    ssd1306_send_cmd(disp, 0xA6); // Normal display
    ssd1306_send_cmd(disp, 0xD5); ssd1306_send_cmd(disp, 0x80); // Clock
    ssd1306_send_cmd(disp, 0x8D); ssd1306_send_cmd(disp, 0x14); // Charge pump
    ssd1306_send_cmd(disp, 0xAF); // Display ON
}

// Atualiza tela com conteúdo do buffer
void ssd1306_show(ssd1306_t *disp) {
    for (uint8_t page = 0; page < 8; page++) {
        ssd1306_send_cmd(disp, 0xB0 + page); // Page address
        ssd1306_send_cmd(disp, 0x00);        // Lower column
        ssd1306_send_cmd(disp, 0x10);        // Higher column
        ssd1306_send_data(disp, &disp->buffer[SSD1306_WIDTH * page], SSD1306_WIDTH);
    }
}

// Limpa tela
void ssd1306_clear(ssd1306_t *disp) {
    memset(disp->buffer, 0, sizeof(disp->buffer));
}

// Seta a tela
void ssd1306_showall(ssd1306_t *disp) {
    memset(disp->buffer, 0xFF, sizeof(disp->buffer));
}

// Desenha um pixel
void ssd1306_draw_pixel(ssd1306_t *disp, int x, int y, bool color) {
    if (x < 0 || x >= SSD1306_WIDTH || y < 0 || y >= SSD1306_HEIGHT) return;
    int byte_index = x + (y / 8) * SSD1306_WIDTH;
    if (color)
        disp->buffer[byte_index] |= (1 << (y & 7));
    else
        disp->buffer[byte_index] &= ~(1 << (y & 7));
}

void ssd1306_draw_char_font(ssd1306_t *disp, int x, int y, char ch, const font_t *font) {
    if (ch < 32 || ch > 127) return;
    int index = (ch) * font->width; // posição do caractere na tabela
    const uint8_t *bitmap = &font->data[index];

    for (int i = 0; i < font->width; i++) {
        for (int j = 0; j < font->height; j++) {
            bool pixel = bitmap[i] & (1 << j);
            ssd1306_draw_pixel(disp, x + i, y + j, pixel);
        }
    }
}

void ssd1306_draw_string_font(ssd1306_t *disp, int x, int y, const char *str, const font_t *font) {
    while (*str) {
        ssd1306_draw_char_font(disp, x, y, *str++, font);
        x += font->width + font->spacing;
    }
}

// Fonte 5x7
extern const uint8_t font5x7[][5];
const font_t FONT_5x7 = {
    .data = (const uint8_t*)font5x7,
    .width = 5,
    .height = 7,
    .spacing = 1
};

// Fonte 8x8
extern const uint8_t font8x8[][8];
const font_t FONT_8x8 = {
    .data = (const uint8_t*)font8x8,
    .width = 8,
    .height = 8,
    .spacing = 1
};

