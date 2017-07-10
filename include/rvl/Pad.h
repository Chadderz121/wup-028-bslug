/* Pad.h
 *   by Alex Chadwick
 *
 * Copyright (C) 2017, Alex Chadwick
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* definitions of symbols inferred to exist in the Pad header file for which the
 * brainslug symbol information is available. */

#ifndef _RVL_PAD_H_
#define _RVL_PAD_H_

#include <stdint.h>

typedef struct PADData_t PADData_t;

void PADInit(void);
void PADRead(PADData_t *result);
void PADControlMotor(int pad, int control);

#define PADData_ERROR_NONE 0
#define PADData_ERROR_NO_CONNECTION -1
#define PADData_ERROR_2 -2
#define PADData_ERROR_3 -3

#define PADData_BUTTON_DL (1 << 0)
#define PADData_BUTTON_DR (1 << 1)
#define PADData_BUTTON_DD (1 << 2)
#define PADData_BUTTON_DU (1 << 3)
#define PADData_BUTTON_Z (1 << 4)
#define PADData_BUTTON_R (1 << 5)
#define PADData_BUTTON_L (1 << 6)
#define PADData_BUTTON_A (1 << 8)
#define PADData_BUTTON_B (1 << 9)
#define PADData_BUTTON_X (1 << 10)
#define PADData_BUTTON_Y (1 << 11)
#define PADData_BUTTON_S (1 << 12)

/* Size 0xc from PADRead */
struct PADData_t {
  uint16_t buttons; /* 0x0 from SPEC2_MakeStatus */
  int8_t aStickX; /* 0x2 from SPEC0_MakeStatus */
  int8_t aStickY; /* 0x3 from SPEC0_MakeStatus */
  int8_t cStickX; /* 0x4 from SPEC0_MakeStatus */
  int8_t cStickY; /* 0x5 from SPEC0_MakeStatus */
  uint8_t sliderL; /* 0x6 from SPEC0_MakeStatus */
  uint8_t sliderR; /* 0x7 from SPEC0_MakeStatus */
  uint8_t _unknown8; /* 0x8 from SPEC0_MakeStatus */
  uint8_t _unknown9; /* 0x9 from SPEC0_MakeStatus */
  int8_t error; /* 0xa from PADRead */
  uint8_t _unknownb[0xc - 0xb];
};

#endif
