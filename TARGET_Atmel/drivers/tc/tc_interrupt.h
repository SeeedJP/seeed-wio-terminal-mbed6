/**
 * \file
 *
 * \brief SAM TC - Timer Counter Callback Driver
 *
 * Copyright (C) 2013-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#ifndef TC_INTERRUPT_H_INCLUDED
#define TC_INTERRUPT_H_INCLUDED

#include <system_interrupt.h>

#ifdef __cplusplus
extern "C" {
#endif

#if !defined(__DOXYGEN__)
extern void *_tc_instances[8];

/**
 * \internal Get the interrupt vector for the given device instance
 *
 * \param[in] TC module instance number.
 *
 * \return Interrupt vector for of the given TC module instance.
 */
static enum system_interrupt_vector _tc_interrupt_get_interrupt_vector(
    uint32_t inst_num)
{
    static uint8_t tc_interrupt_vectors[] = {
        SYSTEM_INTERRUPT_MODULE_TC0,
        SYSTEM_INTERRUPT_MODULE_TC1,
        SYSTEM_INTERRUPT_MODULE_TC2,
        SYSTEM_INTERRUPT_MODULE_TC3,
        SYSTEM_INTERRUPT_MODULE_TC4,
        SYSTEM_INTERRUPT_MODULE_TC5,
        SYSTEM_INTERRUPT_MODULE_TC6,
        SYSTEM_INTERRUPT_MODULE_TC7,
    };

    return (enum system_interrupt_vector)tc_interrupt_vectors[inst_num];
}
#endif /* !defined(__DOXYGEN__) */

/**
 * \name Callback Management
 * {@
 */

enum status_code tc_register_callback(
    struct tc_module *const module,
    tc_callback_t callback_func,
    const enum tc_callback callback_type);

enum status_code tc_unregister_callback(
    struct tc_module *const module,
    const enum tc_callback callback_type);

void tc_enable_callback(
    struct tc_module *const module,
    const enum tc_callback callback_type);

void tc_disable_callback(
    struct tc_module *const module,
    const enum tc_callback callback_type);
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* TC_INTERRUPT_H_INCLUDED */
