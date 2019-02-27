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

#ifndef STEP_28BJY48_H
#define STEP_28BJY48_H

#ifndef PORT_28BJY48
#define PORT_28BJY48 PORTD
#endif /* PORT_28BJY48 */

#define STEPS_28BJY48 2048

struct stepmotor4 {
    uint8_t in[4];
    int8_t s;
};

//struct stepmotor2 {
//    uint8_t in[2];
//    int8_t s;
//};

/* ------------------------------------------------------------------------- */

void sm4_step(struct stepmotor4 *sm, uint8_t hs, uint8_t f);
void sm4_run(struct stepmotor4 *sm, int16_t cnt, uint16_t d, uint8_t hs);

/*
 * sm4_runfs takes a pointer to stepmotor4, number of steps to move (negative
 * numbers move backwards), and delay in ms between steps (recommend staying
 * above 5)
 *
 * sm4_runhs is the same as above but runs the motor in fullsteps instead of
 * half steps, causing the distance travled to be half for the same number of
 * steps
 */
#define sm4_run_fs(sm, c, d) sm4_run(sm, c, d, 0)
#define sm4_run_hs(sm, c, d) sm4_run(sm, c, d, 1)

#endif /* STEP_28BJY48_H */
