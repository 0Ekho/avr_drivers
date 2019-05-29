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
 * https://akizukidenshi.com/download/ds/aosong/DHT11_20180119.pdf
 *
 * // TODO: info
 *
 * give 1 second from powerup
 * do not read within 2 second of last read
 * data is from last read, `read, wait 2s, read` to get data from 2 seconds ago
 * note sensors may take up to 10 seconds to reflect air values (12 seconds ago
 * from read)
 *
 *

 * temp is in decidegree Celsius (10th's of degree Celsius)
 * humidity is in integer % relative
 *
 *
 * Example usage:
 *  struct dht_resp ht;
 *  while (dht_read(&ht, DDD0) < 0) {
 *      // wait 1 second if read is bad, then try again
 *      _delay_ms(1000);
 *      // optionally may want to exit incase of multiple consecutive failed
 *      // reads to prevent getting stuck indefinitly
 *  }
 *  // do something if temp is over 22C
 *  if (ht.temp > 2200) {
 *      foo();
 *  }
 *
 */

#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>

#include "dht11.h"

/* ------------------------------------------------------------------------- */

int dht_read(struct dht_resp *r, uint8_t pin) {
    uint8_t t;
    uint8_t s = 0;
    uint8_t v[5];

    /* page 9 table 10
     * timings are loose enough that it should be safe to ignore the time spent
     * on non delay cycles assuming high clockrate (>8MHz) */

    // TODO: fix timings to allow for lower clockrates
    // (use timer + prescaler for t instead of _delay)

    DDR_DHT11 |= _BV(pin); // set to output

    PORT_DHT11 &= ~_BV(pin); // pull down, start signal
    _delay_ms(20); // min 18, max 30

    DDR_DHT11 &= ~_BV(pin); //set to input
    PORT_DHT11 &= ~_BV(pin); // disable pullups
    _delay_us(35); // make sure bus is released and pulled back down

    // first 2 transistions are not data, next 80 are 5 bytes of data.
    for (int8_t i = -2; i < 80; i++) {
        t = 0;
        while (s == (PIN_DHT11 & _BV(pin))) {
            t++;
            // nothing should take over 92μs, if it does something went wrong
            if (t > 92) {
                return -1;
            }
            _delay_us(1);
        }
        s = PIN_DHT11 & _BV(pin);
        // if s is high, t is for low portion which is always 54, and can be
        // ignored
        if (s > 0 || i < 0) {
            continue;
        }
        // every 2nd transistion should be a bit, so /16
        v[i / 16] <<= 1;
        // if t > 27 signal is "1"
        if (t > 27) {
            v[i / 16] |= 1;
        }
    }
    if (v[0] + v[1] + v[2] + v[3] != v[4]) {
        return -2;
    }

    // note, fractional part needs to be added *after* negating the integer half
    // not before,
    // took almost 200ml of cryogenic rum to figure this out...
    r->temp = v[2] * 10;
    // bottom of page 6.
    // if the high-order bit in v[3] is set then temp is negative
    if (v[3] & 0x80) {
        r->temp = ~r->temp + 1;
    }
    r->temp += (v[3] & 0x7F); // ignore sign bit here

    // v[1] is useless? humidity does not have fractional readings
    r->humi = v[0]; // + v[1]?
    return 1;
}

/* ------------------------------------------------------------------------- */

