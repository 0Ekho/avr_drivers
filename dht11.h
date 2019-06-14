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

#ifndef DHT11_H
#define DHT11_H

#ifndef PORT_DHT11
#define PORT_DHT11 D
#endif /* PORT_DHT11 */
// Is there a better way to do this that still uses defined behavior?
#if PORT_DHT11 == D
#define DHT11_DDR DDRD
#define DHT11_PORT PORTD
#define DHT11_PIN PIND
#elif PORT_DHT11 == C
#define DHT11_DDR DDRC
#define DHT11_PORT PORTC
#define DHT11_PIN PINC
#elif PORT_DHT11 == B
#define DHT11_DDR DDRB
#define DHT11_PORT PORTB
#define DHT11_PIN PINB
#endif /* PORT_DHT11 == {D,C,B} */

/* temp is in decidegree Celsius, humi is in integer %
 * Ex: temp = 205; // 20.5 degree Celsius 
 *     humi = 54; // 54% relative humidity
 * sensor only gives integer humidity values, and temperature is limited to
 * -20-60C, with a single decimal.
 */
struct dht_resp {
    int16_t temp;
    uint8_t humi;
};

/* ------------------------------------------------------------------------- */

int dht_read(struct dht_resp *r, uint8_t pin);

#endif /* DHT11_H */
