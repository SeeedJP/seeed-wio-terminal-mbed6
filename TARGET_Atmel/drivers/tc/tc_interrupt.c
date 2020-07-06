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

#include "tc_drv.h"
#include "tc_interrupt.h"

void *_tc_instances[8];

void _tc_interrupt_handler(uint8_t instance);

/**
 * \brief Registers a callback.
 *
 * Registers a callback function which is implemented by the user.
 *
 * \note The callback must be enabled by \ref tc_enable_callback,
 * in order for the interrupt handler to call it when the conditions for the
 * callback type is met.
 *
 * \param[in]     module      Pointer to TC software instance struct
 * \param[in]     callback_func Pointer to callback function
 * \param[in]     callback_type Callback type given by an enum
 */
enum status_code tc_register_callback(
    struct tc_module *const module,
    tc_callback_t callback_func,
    const enum tc_callback callback_type)
{
    /* Sanity check arguments */
    Assert(module);
    Assert(callback_func);

    /* Register callback function */
    module->callback[callback_type] = callback_func;

    /* Set the bit corresponding to the callback_type */
    if (callback_type == TC_CALLBACK_CC_CHANNEL0) {
        module->register_callback_mask |= TC_INTFLAG_MC(1);
    } else if (callback_type == TC_CALLBACK_CC_CHANNEL1) {
        module->register_callback_mask |= TC_INTFLAG_MC(2);
    } else {
        module->register_callback_mask |= (1 << callback_type);
    }
    return STATUS_OK;
}

/**
 * \brief Unregisters a callback.
 *
 * Unregisters a callback function implemented by the user. The callback should be
 * disabled before it is unregistered.
 *
 * \param[in]     module Pointer to TC software instance struct
 * \param[in]     callback_type Callback type given by an enum
 */
enum status_code tc_unregister_callback(
    struct tc_module *const module,
    const enum tc_callback callback_type)
{
    /* Sanity check arguments */
    Assert(module);

    /* Unregister callback function */
    module->callback[callback_type] = NULL;

    /* Clear the bit corresponding to the callback_type */
    if (callback_type == TC_CALLBACK_CC_CHANNEL0) {
        module->register_callback_mask &= ~TC_INTFLAG_MC(1);
    } else if (callback_type == TC_CALLBACK_CC_CHANNEL1) {
        module->register_callback_mask &= ~TC_INTFLAG_MC(2);
    } else {
        module->register_callback_mask &= ~(1 << callback_type);
    }
    return STATUS_OK;
}

/**
 * \brief Enables callback.
 *
 * Enables the callback function registered by the \ref
 * tc_register_callback. The callback function will be called from the
 * interrupt handler when the conditions for the callback type are
 * met. This function will also enable the appropriate interrupts.
 *
 * \param[in]     module Pointer to TC software instance struct
 * \param[in]     callback_type Callback type given by an enum
 */
void tc_enable_callback(
    struct tc_module *const module,
    const enum tc_callback callback_type)
{
    /* Sanity check arguments */
    Assert(module);

    /* Enable callback */
    if (callback_type == TC_CALLBACK_CC_CHANNEL0) {
        module->enable_callback_mask |= TC_INTFLAG_MC(1);
        module->hw->COUNT8.TC_INTENSET = TC_INTFLAG_MC(1);
    } else if (callback_type == TC_CALLBACK_CC_CHANNEL1) {
        module->enable_callback_mask |= TC_INTFLAG_MC(2);
        module->hw->COUNT8.TC_INTENSET = TC_INTFLAG_MC(2);
    } else {
        module->enable_callback_mask |= (1 << callback_type);
        module->hw->COUNT8.TC_INTENSET = (1 << callback_type);
    }
}

/**
 * \brief Disables callback.
 *
 * Disables the callback function registered by the \ref
 * tc_register_callback, and the callback will not be called from the
 * interrupt routine. The function will also disable the appropriate
 * interrupts.
 *
 * \param[in]     module Pointer to TC software instance struct
 * \param[in]     callback_type Callback type given by an enum
 */
void tc_disable_callback(
    struct tc_module *const module,
    const enum tc_callback callback_type)
{
    /* Sanity check arguments */
    Assert(module);

    /* Disable callback */
    if (callback_type == TC_CALLBACK_CC_CHANNEL0) {
        module->hw->COUNT8.TC_INTENCLR = TC_INTFLAG_MC(1);
        module->enable_callback_mask &= ~TC_INTFLAG_MC(1);
    } else if (callback_type == TC_CALLBACK_CC_CHANNEL1) {
        module->hw->COUNT8.TC_INTENCLR = TC_INTFLAG_MC(2);
        module->enable_callback_mask &= ~TC_INTFLAG_MC(2);
    } else {
        module->hw->COUNT8.TC_INTENCLR = (1 << callback_type);
        module->enable_callback_mask &= ~(1 << callback_type);
    }
}

/**
 * \internal ISR handler for TC
 *
 * Auto-generate a set of interrupt handlers for each TC in the device.
 */
#define _TC_INTERRUPT_HANDLER(n)      \
		void TC##n##_Handler(void)    \
		{                             \
			_tc_interrupt_handler(n); \
		}
_TC_INTERRUPT_HANDLER(0)
_TC_INTERRUPT_HANDLER(1)
_TC_INTERRUPT_HANDLER(2)
_TC_INTERRUPT_HANDLER(3)
_TC_INTERRUPT_HANDLER(4)
_TC_INTERRUPT_HANDLER(5)
_TC_INTERRUPT_HANDLER(6)
_TC_INTERRUPT_HANDLER(7)

/**
 * \internal Interrupt Handler for TC module
 *
 * Handles interrupts as they occur, it will run the callback functions
 * that are registered and enabled.
 *
 * \param[in]  instance  ID of the TC instance calling the interrupt
 *                       handler.
 */
void _tc_interrupt_handler(
    uint8_t instance)
{
    /* Temporary variable */
    uint8_t interrupt_and_callback_status_mask;

    /* Get device instance from the look-up table */
    struct tc_module *module =
        (struct tc_module *)_tc_instances[instance];

    /* Read and mask interrupt flag register */
    interrupt_and_callback_status_mask = module->hw->COUNT8.TC_INTFLAG &
                                         module->register_callback_mask &
                                         module->enable_callback_mask;

    /* Check if an Overflow interrupt has occurred */
    if (interrupt_and_callback_status_mask & TC_INTFLAG_OVF(1)) {
        /* Invoke registered and enabled callback function */
        (module->callback[TC_CALLBACK_OVERFLOW])(module);
        /* Clear interrupt flag */
        module->hw->COUNT8.TC_INTFLAG = TC_INTFLAG_OVF(1);
    }

    /* Check if an Error interrupt has occurred */
    if (interrupt_and_callback_status_mask & TC_INTFLAG_ERR(1)) {
        /* Invoke registered and enabled callback function */
        (module->callback[TC_CALLBACK_ERROR])(module);
        /* Clear interrupt flag */
        module->hw->COUNT8.TC_INTFLAG = TC_INTFLAG_ERR(1);
    }

    /* Check if an Match/Capture Channel 0 interrupt has occurred */
    if (interrupt_and_callback_status_mask & TC_INTFLAG_MC(1)) {
        /* Invoke registered and enabled callback function */
        (module->callback[TC_CALLBACK_CC_CHANNEL0])(module);
        /* Clear interrupt flag */
        module->hw->COUNT8.TC_INTFLAG = TC_INTFLAG_MC(1);
    }

    /* Check if an Match/Capture Channel 1 interrupt has occurred */
    if (interrupt_and_callback_status_mask & TC_INTFLAG_MC(2)) {
        /* Invoke registered and enabled callback function */
        (module->callback[TC_CALLBACK_CC_CHANNEL1])(module);
        /* Clear interrupt flag */
        module->hw->COUNT8.TC_INTFLAG = TC_INTFLAG_MC(2);
    }
}
