/* BSD-0
 Copyright (C) 2019, Ekho <ekho@ekho.email>

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 ------------------------------------------------------------------------------
*/

#ifndef LCD_H
#define LCD_H

#ifndef LCD_CTRL_PORT
#define LCD_CTRL_PORT PORTB
#endif /* LCD_CTRL_PORT */

#ifndef LCD_DATA_PORT
#define LCD_DATA_PORT PORTD
#endif /* LCD_DATA_PORT */

/* Instruction information availible on the HD44780 datasheet
 * Page 24, Table 6
 * Define naming is consistent with Arduino LiquidCrystal to ease conversion
 * from Arduino -> AVR.
 */
// command bits
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80
// display entry modes
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00
// display option enable/disable
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00
// display/cursor movement
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00
// function settings
//#define LCD_8BITMODE 0x10 // 8bit mode not supported for now
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

/* ------------------------------------------------------------------------- */

struct lcd {
    uint8_t rs;
    uint8_t e;
    uint8_t d[4];
    uint8_t function;
    uint8_t control;
    uint8_t mode;
};

/* ------------------------------------------------------------------------- */

void lcd_init(struct lcd *disp, uint8_t rs, uint8_t e, uint8_t d4, uint8_t d5,
    uint8_t d6, uint8_t d7, uint8_t func);
// Lower level commands incase of missing user commands
void lcd_cmd(struct lcd *disp, uint8_t val);
void lcd_write(struct lcd *disp, uint8_t val);

// User commands for convinience
void lcd_print(struct lcd *disp, const uint8_t *data);
void lcd_clear(struct lcd *disp);
void lcd_home(struct lcd *disp);
void lcd_set_cursor(struct lcd *disp, uint8_t col, uint8_t row);
void lcd_display_on(struct lcd *disp);
void lcd_display_off(struct lcd *disp);

/* ------------------------------------------------------------------------- */

#endif /* LCD_H */
