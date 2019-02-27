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

/* basic driver for 28BYJ-48 5V stepper motor
 *
 * 4 pin connections are supported via the sm4 set of functions / macros.
 * Full stepping and half stepping (4 pin only)
 *
 * STEPS_28BJY48 is defined to 1 full rotation (2048 steps)
 *
 * Stepper pins in[1-4] MUST share a port, by default this is PORTD
 * PORT_28BJY48 may be defined in your makefile to change this.
 *
 * Recommend power to the stepper motor is cut when not in use. (put it on a
 * relay, etc.)
 *
 * Example usage:
 *  // set pins                   IN1     IN2     IN3     IN4
 *  struct stepmotor4 stepper = {{PORTD0, PORTD1, PORTD2, PORTD3}, 0};
 *  DDRD |= _BV(DDD0) | _BV(DDD1) | _BV(DDD2) | _BV(DDD3);
 *  //                   steps to run, ~ms delay between steps
 *  sm4_run_fs(&stepper, STEPS_28BJY48, 10);
 *  // ^ this causes one full revolution forward
 *
 */

// TODO: 2 pin connection support
// TODO: shiftregister support for driving multiple steppers at once
#include <avr/io.h>
#include <stdint.h>
#include <stdlib.h>
#include <util/delay.h>

#include "28bjy48.h"

/* ------------------------------------------------------------------------- */

// Even index is full steps
static const uint8_t steps_28bjy48[8] = {
// forwards  4321  
    0x03, // 0011
    0x02, // 0010
    0x06, // 0110
    0x04, // 0100
    0x0C, // 1100
    0x08, // 1000
    0x09, // 1001
    0x01, // 0001
// backwards 1234
};

/* ------------------------------------------------------------------------- */

void sm4_step(struct stepmotor4 *sm, uint8_t hs, uint8_t f)
{
    for (uint8_t i = 0; i < 4; i++)  {
        // reverse pinout of stepper to run backwards
        uint8_t j = f ? i : (3 - i);
        if ((steps_28bjy48[sm->s] >> i) & 0x01) {
            PORT_28BJY48 |= _BV(sm->in[j]);
        } else {
            PORT_28BJY48 &= ~_BV(sm->in[j]);
        }
    }
    sm->s = (sm->s + (hs ? 1 : 2)) % 8;
}

void sm4_run(struct stepmotor4 *sm, int16_t cnt, uint16_t d, uint8_t hs)
{
    for (int16_t i = 0; i < abs(cnt); i++) {
        sm4_step(sm, hs, (cnt > 0));
        for (uint16_t j = 0; j < d; j++) {
            _delay_ms(1);
        }
    }
}
