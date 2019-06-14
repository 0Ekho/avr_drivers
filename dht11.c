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

/* driver for DHT11 temperature & humidity sensor
 *
 * Datasheet (English):
 * 'https://akizukidenshi.com/download/ds/aosong/DHT11_20180119.pdf'
 *
 * - temp is in decidegree Celsius (10th's of degree Celsius)
 * - humidity is in integer % relative
 * - by default uses PORTD, PORT_DHT11 may be defined in your makefile to change
 *   this, note that only the letter is used, the string "PORT" is omitted 
 *   (Ex: -DPORT_DHT11=B)
 * - timer0 is used while inside dht_read(), additionally the timer is not
 *   disabled or restored to its previous state after use
 *
 * Extra notes:
 * - give 1 second from powerup before reading.
 * - do not read within 2 second of last successful read, with the exception of
 *   doing a double read to get mostly realtime data (not 100% sure on this,
 *   also unsure is this actually applies to only successful reads or if failed
 *   reads also need a delay)
 * - data is from last read, so if reads are spaced far apart will have old
 *   data (read twice to get new data)
 * - not tested below 1MHz currently, but should work down to 400kHz, possibly
 *   lower
 *
 *
 * Example usage:
 *  struct dht_resp ht;
 *
 *  // wait 1s after powerup
 *  _delay_ms(1000);
 *
 *  if (dht_read(&ht, DDD0) > 0) {
 *      // call foo() if temp is over 22C
 *      if (ht.temp > 220) {
 *          foo();
 *      }
 *  } else {
 *      // read failed, handle this however
 *      return -1;
 *  }
 *
 */

#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>

#include "dht11.h"

/* Page 9, Table 10, */
#define DHT_T_LOW 27 // upper bound of low pulse
//#define DHT_T_HIGH 68 // lower bound of high pulse
#define DHT_TIMEOUT 92 // highest value in timing table

/* use prescaler over 2MHz to fit in 8bit timer */
#define DHT_PRESCALE_F 2000000UL // 2MHz
#define DHT_T_SCALE 1000000.0 // timings are in microseconds

/* ------------------------------------------------------------------------- */

int dht_read(struct dht_resp *r, uint8_t pin) {
    uint8_t s = 0;
    uint8_t v[5];

    /* Page 9 Table 10 of datasheet for timings */

    DHT11_DDR |= _BV(pin); // set to output

    DHT11_PORT &= ~_BV(pin); // pull down, start signal
    _delay_ms(20); // min 18, max 30

    DHT11_DDR &= ~_BV(pin); //set to input
    DHT11_PORT &= ~_BV(pin); // disable pullups
    _delay_us(35); // make sure bus is released and pulled back down

    // no interrupts on timer
    TIMSK0 &= (~_BV(OCIE0B) & ~_BV(OCIE0B) & ~_BV(TOIE0));
    // set normal mode
    TCCR0A &= (~_BV(WGM00) & ~_BV(WGM01));
#if F_CPU > DHT_PRESCALE_F
    // enable d8 prescaler
    TCCR0B &= (~_BV(CS00) & ~_BV(CS02) & ~_BV(WGM02));
    TCCR0B |= _BV(CS01);
    // calculate the timer values needed for $time, erroring on the higher side
#define DHT_TIMEOUT_T (uint8_t)ceil(((DHT_TIMEOUT / DHT_T_SCALE) * F_CPU) / 8.0)
#define DHT_T_LOW_T (uint8_t)ceil(((DHT_T_LOW / DHT_T_SCALE) * F_CPU) / 8.0)
//#define DHT_T_HIGH_T (uint8_t)ceil(((DHT_T_HIGH / DHT_T_SCALE) * F_CPU) / 8.0)
#else
    // no prescaling
    TCCR0B &= (~_BV(CS01) & ~_BV(CS02) & ~_BV(WGM02));
    TCCR0B |= _BV(CS00);
#define DHT_TIMEOUT_T (uint8_t)ceil((DHT_TIMEOUT / DHT_T_SCALE) * F_CPU)
#define DHT_T_LOW_T (uint8_t)ceil((DHT_T_LOW / DHT_T_SCALE) * F_CPU)
//#define DHT_T_HIGH_T (uint8_t)ceil((DHT_T_HIGH / DHT_T_SCALE) * F_CPU)
#endif /* F_CPU > DHT_PRESCALE_F */

    // first 2 transitions are not data, next 80 are 5 bytes of data.
    for (int8_t i = -2; i < 80; i++) {
        uint8_t t;

        TCNT0 = 0;
        while (s == (DHT11_PIN & _BV(pin))) {
            // nothing should take over 92μs, if it does something went wrong
            if (TCNT0 > DHT_TIMEOUT_T) {
                return -1;
            }
        }
        t = TCNT0;

        s = DHT11_PIN & _BV(pin);
        // if s is high, t is for low portion which is always 54μs, and can be
        // ignored
        if (s > 0 || i < 0) {
            continue;
        }
        // every 2nd transition should be a bit, so /16
        v[i / 16] <<= 1;
        // if t > 27μs signal is "1"
        if (t > DHT_T_LOW_T) {
            v[i / 16] |= 1;
        }
    }
    if (v[0] + v[1] + v[2] + v[3] != v[4]) {
        return -2;
    }

    /* note: fractional part needs to be added *after* negating the integer half
     * not before, (integer part is always lower than (or equal to) temp, adding
     * fractional part to bring it up to the correct value)
     * took almost 200ml of cryogenic rum to figure this out... */
    r->temp = v[2] * 10;
    // bottom of page 6.
    // if the high-order bit in v[3] is set then temp is negative
    if (v[3] & 0x80) {
        r->temp = ~r->temp + 1;
    }
    r->temp += (v[3] & 0x7F); // ignore sign bit here

    // v[1] is useless? Humidity does not have fractional readings
    r->humi = v[0];
    return 1;
}
