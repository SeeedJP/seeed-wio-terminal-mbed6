/*
 *
 *  Copyright (C) 2006-2015, ARM Limited, All Rights Reserved
 *  SPDX-License-Identifier: Apache-2.0
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may
 *  not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 */

#include <stdlib.h>
#include "trng_api.h"
#include "cmsis.h"
#include "trng.h"

void trng_init(trng_t *obj)
{
    (void)obj;

    /* Enable the user interface clock in the PM */
    system_apb_clock_set_mask(SYSTEM_CLOCK_APB_APBC, MCLK_APBCMASK_TRNG(1));

    TRNG_REGS->TRNG_CTRLA = TRNG_CTRLA_ENABLE(1);
}

void trng_free(trng_t *obj)
{
    (void)obj;

    TRNG_REGS->TRNG_CTRLA = TRNG_CTRLA_ENABLE(0);
}

int trng_get_bytes(trng_t *obj, uint8_t *output, size_t length, size_t *output_length)
{
    (void)obj;

    *output_length = length;

    uint32_t *out32 = (uint32_t *)output;
    while (length >= 4) {
        while (!(TRNG_REGS->TRNG_INTFLAG & TRNG_INTFLAG_DATARDY(1))) {};
        *out32++ = TRNG_REGS->TRNG_DATA;
        length -= 4;
    }

    if (length > 0) {
        while (!(TRNG_REGS->TRNG_INTFLAG & TRNG_INTFLAG_DATARDY(1))) {};
        uint32_t rn = TRNG_REGS->TRNG_DATA;
        uint8_t *out8 = (uint8_t *)out32;

        while (length > 0) {
            *out8++ = (uint8_t)(rn & 0xff);
            rn >>= 8;
            length--;
        }
    }

    return 0;
}
