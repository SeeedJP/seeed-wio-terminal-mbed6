/**
 * \file
 *
 * \brief SAM Power related functionality
 *
 * Copyright (C) 2014-2015 Atmel Corporation. All rights reserved.
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
#ifndef POWER_H_INCLUDED
#define POWER_H_INCLUDED

#include <compiler.h>
#include <cmsis.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \addtogroup asfdoc_sam0_system_group
 * @{
 */

/**
 * \brief Voltage references within the device.
 *
 * List of available voltage references (VREF) that may be used within the
 * device.
 */
enum system_voltage_reference {
    /** Temperature sensor voltage reference. */
    SYSTEM_VOLTAGE_REFERENCE_TEMPSENSE,
    /** Bandgap voltage reference. */
    SYSTEM_VOLTAGE_REFERENCE_BANDGAP,
};

/**
 * \brief Device sleep modes.
 *
 * List of available sleep modes in the device. A table of clocks available in
 * different sleep modes can be found in \ref asfdoc_sam0_system_module_overview_sleep_mode.
 */
enum system_sleepmode {
    SYSTEM_SLEEPMODE_IDLE = PM_SLEEPCFG_SLEEPMODE_IDLE,
    SYSTEM_SLEEPMODE_STANDBY = PM_SLEEPCFG_SLEEPMODE_STANDBY,
    SYSTEM_SLEEPMODE_HIBERNATE = PM_SLEEPCFG_SLEEPMODE_HIBERNATE,
    SYSTEM_SLEEPMODE_BACKUP = PM_SLEEPCFG_SLEEPMODE_BACKUP,
    SYSTEM_SLEEPMODE_OFF = PM_SLEEPCFG_SLEEPMODE_OFF
};

/**
 * \name Voltage References
 * @{
 */

/**
 * \brief Enable the selected voltage reference
 *
 * Enables the selected voltage reference source, making the voltage reference
 * available on a pin as well as an input source to the analog peripherals.
 *
 * \param[in] vref  Voltage reference to enable
 */
static inline void system_voltage_reference_enable(
    const enum system_voltage_reference vref)
{
    switch (vref) {
        case SYSTEM_VOLTAGE_REFERENCE_TEMPSENSE:
            SUPC_REGS->SUPC_VREF |= SUPC_VREF_TSEN(1);
            break;

        case SYSTEM_VOLTAGE_REFERENCE_BANDGAP:
            SUPC_REGS->SUPC_VREF |= SUPC_VREF_VREFOE(1);
            break;

        default:
            Assert(false);
            return;
    }
}

/**
 * \brief Disable the selected voltage reference
 *
 * Disables the selected voltage reference source.
 *
 * \param[in] vref  Voltage reference to disable
 */
static inline void system_voltage_reference_disable(
    const enum system_voltage_reference vref)
{
    switch (vref) {
        case SYSTEM_VOLTAGE_REFERENCE_TEMPSENSE:
            SUPC_REGS->SUPC_VREF &= ~SUPC_VREF_TSEN(1);
            break;

        case SYSTEM_VOLTAGE_REFERENCE_BANDGAP:
            SUPC_REGS->SUPC_VREF &= ~SUPC_VREF_VREFOE(1);
            break;

        default:
            Assert(false);
            return;
    }
}

/**
 * @}
 */

/**
 * \name Device Sleep Control
 * @{
 */

/**
 * \brief Set the sleep mode of the device
 *
 * Sets the sleep mode of the device; the configured sleep mode will be entered
 * upon the next call of the \ref system_sleep() function.
 *
 * For an overview of which systems are disabled in sleep for the different
 * sleep modes, see \ref asfdoc_sam0_system_module_overview_sleep_mode.
 *
 * \param[in] sleep_mode  Sleep mode to configure for the next sleep operation
 *
 * \retval STATUS_OK               Operation completed successfully
 * \retval STATUS_ERR_INVALID_ARG  The requested sleep mode was invalid or not
 *                                 available
 */
static inline enum status_code system_set_sleepmode(
    const enum system_sleepmode sleep_mode)
{
    /* Make sure that the Flash does not power all the way down
     * when in sleep mode. */
    NVMCTRL_REGS->NVMCTRL_CTRLA &= ~NVMCTRL_CTRLA_PRM_Msk;
    NVMCTRL_REGS->NVMCTRL_CTRLA |= NVMCTRL_CTRLA_PRM_FULLAUTO;

    switch (sleep_mode) {
        case SYSTEM_SLEEPMODE_IDLE:
        case SYSTEM_SLEEPMODE_OFF:
            SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
            PM_REGS->PM_SLEEPCFG = sleep_mode;
            break;

        case SYSTEM_SLEEPMODE_STANDBY:
        case SYSTEM_SLEEPMODE_HIBERNATE:
        case SYSTEM_SLEEPMODE_BACKUP:
            SCB->SCR |=  SCB_SCR_SLEEPDEEP_Msk;
            PM_REGS->PM_SLEEPCFG = sleep_mode;
            break;

        default:
            return STATUS_ERR_INVALID_ARG;
    }

    return STATUS_OK;
}

/**
 * \brief Put the system to sleep waiting for interrupt
 *
 * Executes a device DSB (Data Synchronization Barrier) instruction to ensure
 * all ongoing memory accesses have completed, then a WFI (Wait For Interrupt)
 * instruction to place the device into the sleep mode specified by
 * \ref system_set_sleepmode until woken by an interrupt.
 */
static inline void system_sleep(void)
{
    __DSB();
    __WFI();
}

/**
 * @}
 */

/** @} */
#ifdef __cplusplus
}
#endif

#endif /* POWER_H_INCLUDED */
