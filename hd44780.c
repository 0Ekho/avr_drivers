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

/* basic driver for HD44780 LCD displays with ATmega328p,
 *
 * RS & E MUST be on the same port, D[4-7] MUST also share a port
 * (however RS/E and D[n] may be on seperate ports)
 *
 * LCD_CTRL_PORT and LCD_DATA_PORT may be defined in your makefile to control
 * which ports are used, defaults to PORTB and PORTD respectivly.
 *
 *
 * Example Usage:
 *
 *  #include "avr_drivers/hd44780.h"
 *
 *  struct lcd my_lcd = {0};
 *  DDRB |= _BV(DDB3) | _BV(DDB4);
 *  DDRD |= _BV(DDD2) | _BV(DDD3) | _BV(DDD4) | _BV(DDD5);
 *
 *  // lcd, RS, E, D4, D5, D6, D7, function setting
 *  lcd_init(&my_lcd, PORTB4, PORTB3, PORTD5, PORTD4, PORTD3, PORTD2,
 *      LCD_2LINE | LCD_5x8DOTS);
 *
 *  lcd_print(&my_lcd, (uint8_t *)"Hello World");
*/

#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>

#include "hd44780.h"

/* ------------------------------------------------------------------------- */

void lcd_write_4pin(struct lcd *disp, uint8_t val);

/* ------------------------------------------------------------------------- */

void lcd_write_4pin(struct lcd *disp, uint8_t val)
{
    for (int i = 0; i < 4; i++) {
        if (val >> i & 0x01) {
           LCD_DATA_PORT |= _BV(disp->d[i]);
        } else {
           LCD_DATA_PORT &= ~_BV(disp->d[i]);
        }
    }
    _delay_us(1);
    LCD_CTRL_PORT &= ~_BV(disp->e);
    _delay_us(1);
    LCD_CTRL_PORT |= _BV(disp->e);
    _delay_us(1);
    LCD_CTRL_PORT &= ~_BV(disp->e);
    // Needs > 37us for most commands etc. just wait here to make things simple
    _delay_us(40);
}

void lcd_cmd(struct lcd *disp, uint8_t val)
{
    LCD_CTRL_PORT &= ~_BV(disp->rs);
    lcd_write_4pin(disp, val >> 4);
    lcd_write_4pin(disp, val & 0x0F);
}

void lcd_write(struct lcd *disp, uint8_t val)
{
    LCD_CTRL_PORT |= _BV(disp->rs);
    lcd_write_4pin(disp, val >> 4);
    lcd_write_4pin(disp, val & 0x0F);
}

/* ------------------------------------------------------------------------- */

void lcd_init(struct lcd *disp, uint8_t rs, uint8_t e, uint8_t d4, uint8_t d5,
    uint8_t d6, uint8_t d7, uint8_t func)
{
    disp->rs = rs;
    disp->e = e;
    disp->d[0] = d4;
    disp->d[1] = d5;
    disp->d[2] = d6;
    disp->d[3] = d7;

    // Initialization specification on Hitachi HD44780 datasheet Page 45/46
    _delay_ms(45);
    // Pull both RS low to begin commands
    LCD_CTRL_PORT &= ~_BV(disp->rs);
    // Make sure Enable is low also
    LCD_CTRL_PORT &= ~_BV(disp->e);
    
    // For now only supporting the 4 bit interface.
    // Configuration flowchart, Hitachi HD44780 datasheet Page 46, Figure 24
    lcd_write_4pin(disp, 0x03);
    _delay_us(4200);
    lcd_write_4pin(disp, 0x03);
    _delay_us(4200);
    lcd_write_4pin(disp, 0x03);
    _delay_us(150);
    lcd_write_4pin(disp, 0x02);

    // Set function and defaults incase MCU was reset without powering off
    // display
    disp->function = func | LCD_4BITMODE;
    lcd_cmd(disp, LCD_FUNCTIONSET | disp->function);

    disp->control = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
    lcd_cmd(disp, LCD_DISPLAYCONTROL | disp->control);

    lcd_cmd(disp, LCD_CLEARDISPLAY);
    // ^^ slow command, see page 24 of datasheet for timings
    _delay_us(1550);

    disp->mode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
    lcd_cmd(disp, LCD_ENTRYMODESET | disp->mode);
}

/* ------------------------------------------------------------------------- */
/* user commands */

void lcd_print(struct lcd *disp, const uint8_t *data)
{
    while (*data > '\0') {
        lcd_write(disp, *data);
        data++;
    }
}

void lcd_clear(struct lcd *disp)
{
    lcd_cmd(disp, LCD_CLEARDISPLAY);
    // ^^ slow command, see page 24 of datasheet for timings
    _delay_us(1550);
}
void lcd_home(struct lcd *disp)
{
    lcd_cmd(disp, LCD_RETURNHOME);
    // ^^ slow command, see page 24 of datasheet for timings
    _delay_us(1550);
}
// Caution, no protection against OOB cursor col / row.
void lcd_set_cursor(struct lcd *disp, uint8_t col, uint8_t row)
{
    // Hitachi HD44780 datasheet page 10 for DDRAM layout
    // not sure if this supports RTL text, think memory changes
    // TODO: make sure RTL text works / add support if needed
    char ofst[] = {0x00, 0x40};
    lcd_cmd(disp, LCD_SETDDRAMADDR | (col + ofst[row]));
}
void lcd_display_on(struct lcd *disp)
{
    disp->control &= ~LCD_DISPLAYON;
    lcd_cmd(disp, LCD_DISPLAYCONTROL | disp->control);
}
void lcd_display_off(struct lcd *disp)
{
    disp->control |= LCD_DISPLAYON;
    lcd_cmd(disp, LCD_DISPLAYCONTROL | disp->control);
}
