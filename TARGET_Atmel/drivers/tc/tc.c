/**
 * \file
 *
 * \brief SAM TC - Timer Counter Driver
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
#include "cmsis.h"
#include "tc_drv.h"

#if TC_ASYNC == true
#  include "tc_interrupt.h"
#  include <system_interrupt.h>
#endif

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

/**
 * \internal Find the index of given TC module instance.
 *
 * \param[in] TC module instance pointer.
 *
 * \return Index of the given TC module instance.
 */
uint8_t _tc_get_inst_index(
    tc_registers_t *const hw)
{
    /* List of available TC modules. */
    tc_registers_t *const tc_modules[] = {
        TC0_REGS,
        TC1_REGS,
        TC2_REGS,
        TC3_REGS,
        TC4_REGS,
        TC5_REGS,
        TC6_REGS,
        TC7_REGS
    };

    /* Find index for TC instance. */
    for (uint32_t i = 0; i < ARRAY_SIZE(tc_modules); i++) {
        if (hw == tc_modules[i]) {
            return i;
        }
    }

    /* Invalid data given. */
    Assert(false);
    return 0;
}


/**
 * \brief Initializes a hardware TC module instance.
 *
 * Enables the clock and initializes the TC module, based on the given
 * configuration values.
 *
 * \param[in,out] module_inst  Pointer to the software module instance struct
 * \param[in]     hw           Pointer to the TC hardware module
 * \param[in]     config       Pointer to the TC configuration options struct
 *
 * \return Status of the initialization procedure.
 *
 * \retval STATUS_OK           The module was initialized successfully
 * \retval STATUS_BUSY         Hardware module was busy when the
 *                             initialization procedure was attempted
 * \retval STATUS_INVALID_ARG  An invalid configuration option or argument
 *                             was supplied
 * \retval STATUS_ERR_DENIED   Hardware module was already enabled, or the
 *                             hardware module is configured in 32-bit
 *                             slave mode
 */
enum status_code tc_init(
    struct tc_module *const module_inst,
    tc_registers_t *const hw,
    const struct tc_config *const config)
{
    /* Sanity check arguments */
    Assert(hw);
    Assert(module_inst);
    Assert(config);

    /* Temporary variable to hold all updates to the CTRLA
     * register before they are written to it */
    uint16_t ctrla_tmp = 0;
    /* Temporary variable to hold all updates to the CTRLBSET
     * register before they are written to it */
    uint8_t ctrlbset_tmp = 0;
    /* Temporary variable to hold all updates to the DRVCTRL
     * register before they are written to it */
    uint8_t drvctrl_tmp = 0;
    /* Temporary variable to hold TC instance number */
    uint8_t instance = _tc_get_inst_index(hw);

    /* Array of GLCK ID for different TC instances */
    uint8_t gclk_chan_id[] = {
        GCLK_PCHCTRL_ID_9_GCLK_TC0,
        GCLK_PCHCTRL_ID_9_GCLK_TC1,
        GCLK_PCHCTRL_ID_26_GCLK_TC2,
        GCLK_PCHCTRL_ID_26_GCLK_TC3,
        GCLK_PCHCTRL_ID_30_GCLK_TC4,
        GCLK_PCHCTRL_ID_30_GCLK_TC5,
        GCLK_PCHCTRL_ID_39_GCLK_TC6,
        GCLK_PCHCTRL_ID_39_GCLK_TC7,
    };
    /* Array of PM APBC mask bit position for different TC instances */
    struct tc_apb_mask {
        enum system_clock_apb_bus apb_bus;
        uint32_t mask;
    } tc_apb[] = {
        { SYSTEM_CLOCK_APB_APBA, MCLK_APBAMASK_TC0(1), },
        { SYSTEM_CLOCK_APB_APBA, MCLK_APBAMASK_TC1(1), },
        { SYSTEM_CLOCK_APB_APBB, MCLK_APBBMASK_TC2(1), },
        { SYSTEM_CLOCK_APB_APBB, MCLK_APBBMASK_TC3(1), },
        { SYSTEM_CLOCK_APB_APBC, MCLK_APBCMASK_TC4(1), },
        { SYSTEM_CLOCK_APB_APBC, MCLK_APBCMASK_TC5(1), },
        { SYSTEM_CLOCK_APB_APBD, MCLK_APBDMASK_TC6(1), },
        { SYSTEM_CLOCK_APB_APBD, MCLK_APBDMASK_TC7(1), },
    };

    struct system_pinmux_config pin_config;
    struct system_gclk_chan_config gclk_chan_config;

#if TC_ASYNC == true
    /* Initialize parameters */
    for (uint8_t i = 0; i < TC_CALLBACK_N; i++) {
        module_inst->callback[i]        = NULL;
    }
    module_inst->register_callback_mask     = 0x00;
    module_inst->enable_callback_mask       = 0x00;

    /* Register this instance for callbacks*/
    _tc_instances[instance] = module_inst;
#endif

    /* Associate the given device instance with the hardware module */
    module_inst->hw = hw;

    /* Make the counter size variable in the module_inst struct reflect
     * the counter size in the module
     */
    module_inst->counter_size = config->counter_size;

    if (hw->COUNT8.TC_CTRLA & TC_CTRLA_SWRST(1)) {
        /* We are in the middle of a reset. Abort. */
        return STATUS_BUSY;
    }

    if (hw->COUNT8.TC_STATUS & TC_STATUS_SLAVE(1)) {
        /* Module is used as a slave */
        return STATUS_ERR_DENIED;
    }

    if (hw->COUNT8.TC_CTRLA & TC_CTRLA_ENABLE(1)) {
        /* Module must be disabled before initialization. Abort. */
        return STATUS_ERR_DENIED;
    }

    /* Set up the TC PWM out pin for channel 0 */
    if (config->pwm_channel[0].enabled) {
        system_pinmux_get_config_defaults(&pin_config);
        pin_config.mux_position = config->pwm_channel[0].pin_mux;
        pin_config.direction = SYSTEM_PINMUX_PIN_DIR_OUTPUT;
        system_pinmux_pin_set_config(
            config->pwm_channel[0].pin_out, &pin_config);
    }

    /* Set up the TC PWM out pin for channel 1 */
    if (config->pwm_channel[1].enabled) {
        system_pinmux_get_config_defaults(&pin_config);
        pin_config.mux_position = config->pwm_channel[1].pin_mux;
        pin_config.direction = SYSTEM_PINMUX_PIN_DIR_OUTPUT;
        system_pinmux_pin_set_config(
            config->pwm_channel[1].pin_out, &pin_config);
    }

    /* Enable the user interface clock in the PM */
    system_apb_clock_set_mask(tc_apb[instance].apb_bus, tc_apb[instance].mask);
    if (config->counter_size == TC_COUNTER_SIZE_32BIT) {
        system_apb_clock_set_mask(tc_apb[instance+1].apb_bus, tc_apb[instance+1].mask);
    }

    /* Setup clock for module */
    system_gclk_chan_get_config_defaults(&gclk_chan_config);
    gclk_chan_config.source_generator = config->clock_source;
    system_gclk_chan_set_config(gclk_chan_id[instance], &gclk_chan_config);
    system_gclk_chan_enable(gclk_chan_id[instance]);

    /* Set ctrla register */
    ctrla_tmp =
        (uint32_t)config->counter_size |
        (uint32_t)config->reload_action |
        (uint32_t)config->clock_prescaler;

    if (config->clock_prescaler) {
        ctrla_tmp |= TC_CTRLA_PRESCSYNC_PRESC;
    }

    if (config->run_in_standby) {
        ctrla_tmp |= TC_CTRLA_RUNSTDBY(1);
    }

    for (uint8_t i = 0; i < NUMBER_OF_COMPARE_CAPTURE_CHANNELS; i++) {
        if (config->enable_capture_on_channel[i] == true) {
            ctrla_tmp |= (TC_CTRLA_CAPTEN(1) << i);
        }
    }

    while (tc_is_syncing(module_inst)) {
        /* Wait for sync */
    }

    /* Write configuration to register */
    hw->COUNT8.TC_CTRLA = ctrla_tmp;
    while (tc_is_syncing(module_inst)) {
        /* Wait for sync */
    }

    hw->COUNT8.TC_WAVE = config->wave_generation;

    /* Set ctrlb register */
    if (config->oneshot) {
        ctrlbset_tmp = TC_CTRLBSET_ONESHOT(1);
    }

    if (config->count_direction) {
        ctrlbset_tmp |= TC_CTRLBSET_DIR(1);
    }

    /* Clear old ctrlb configuration */
    hw->COUNT8.TC_CTRLBCLR = TC_CTRLBCLR_Msk;
    while (tc_is_syncing(module_inst)) {
        /* Wait for sync */
    }

    /* Check if we actually need to go into a wait state. */
    if (ctrlbset_tmp) {
        /* Write configuration to register */
        hw->COUNT8.TC_CTRLBSET = ctrlbset_tmp;
        while (tc_is_syncing(module_inst)) {
            /* Wait for sync */
        }
    }

    /* Set drvctrl register*/
    for (uint8_t i = 0; i < NUMBER_OF_COMPARE_CAPTURE_CHANNELS; i++) {
        if (config->enable_capture_on_channel[i] == true) {
            drvctrl_tmp |= (TC_DRVCTRL_INVEN(1) << i);
        }
    }

    /* Write configuration to register */
    hw->COUNT8.TC_DRVCTRL = drvctrl_tmp;

    /* Write configuration to register */
    while (tc_is_syncing(module_inst)) {
        /* Wait for sync */
    }

    /* Switch for TC counter size  */
    switch (module_inst->counter_size) {
        case TC_COUNTER_SIZE_8BIT:
            hw->COUNT8.TC_COUNT =
                config->counter_8_bit.value;
            while (tc_is_syncing(module_inst)) {
                /* Wait for sync */
            }

            hw->COUNT8.TC_PER =
                config->counter_8_bit.period;
            while (tc_is_syncing(module_inst)) {
                /* Wait for sync */
            }

            hw->COUNT8.TC_CC[0] =
                config->counter_8_bit.compare_capture_channel[0];
            while (tc_is_syncing(module_inst)) {
                /* Wait for sync */
            }

            hw->COUNT8.TC_CC[1] =
                config->counter_8_bit.compare_capture_channel[1];
            while (tc_is_syncing(module_inst)) {
                /* Wait for sync */
            }

            return STATUS_OK;

        case TC_COUNTER_SIZE_16BIT:
            hw->COUNT16.TC_COUNT
                = config->counter_16_bit.value;
            while (tc_is_syncing(module_inst)) {
                /* Wait for sync */
            }

            hw->COUNT16.TC_CC[0] =
                config->counter_16_bit.compare_capture_channel[0];
            while (tc_is_syncing(module_inst)) {
                /* Wait for sync */
            }

            hw->COUNT16.TC_CC[1] =
                config->counter_16_bit.compare_capture_channel[1];
            while (tc_is_syncing(module_inst)) {
                /* Wait for sync */
            }

            return STATUS_OK;

        case TC_COUNTER_SIZE_32BIT:
            hw->COUNT32.TC_COUNT
                = config->counter_32_bit.value;
            while (tc_is_syncing(module_inst)) {
                /* Wait for sync */
            }

            hw->COUNT32.TC_CC[0] =
                config->counter_32_bit.compare_capture_channel[0];
            while (tc_is_syncing(module_inst)) {
                /* Wait for sync */
            }

            hw->COUNT32.TC_CC[1] =
                config->counter_32_bit.compare_capture_channel[1];
            while (tc_is_syncing(module_inst)) {
                /* Wait for sync */
            }

            return STATUS_OK;
    }

    Assert(false);
    return STATUS_ERR_INVALID_ARG;
}

/**
 * \brief Sets TC module count value.
 *
 * Sets the current timer count value of a initialized TC module. The
 * specified TC module may be started or stopped.
 *
 * \param[in] module_inst  Pointer to the software module instance struct
 * \param[in] count        New timer count value to set
 *
 * \return Status of the count update procedure.
 *
 * \retval STATUS_OK               The timer count was updated successfully
 * \retval STATUS_ERR_INVALID_ARG  An invalid timer counter size was specified
 */
enum status_code tc_set_count_value(
    const struct tc_module *const module_inst,
    const uint32_t count)
{
    /* Sanity check arguments */
    Assert(module_inst);
    Assert(module_inst->hw);

    /* Get a pointer to the module's hardware instance*/
    tc_registers_t *const tc_module = module_inst->hw;

    /* Write to based on the TC counter_size */
    switch (module_inst->counter_size) {
        case TC_COUNTER_SIZE_8BIT:
            tc_module->COUNT8.TC_COUNT  = (uint8_t)count;
            break;

        case TC_COUNTER_SIZE_16BIT:
            tc_module->COUNT16.TC_COUNT = (uint16_t)count;
            break;

        case TC_COUNTER_SIZE_32BIT:
            tc_module->COUNT32.TC_COUNT = (uint32_t)count;
            break;

        default:
            return STATUS_ERR_INVALID_ARG;
    }

    while (tc_is_syncing(module_inst)) {
        /* Wait for sync */
    }

    return STATUS_OK;
}

/**
 * \brief Get TC module count value.
 *
 * Retrieves the current count value of a TC module. The specified TC module
 * may be started or stopped.
 *
 * \param[in] module_inst  Pointer to the software module instance struct
 *
 * \return Count value of the specified TC module.
 */
uint32_t tc_get_count_value(
    const struct tc_module *const module_inst)
{
    /* Sanity check arguments */
    Assert(module_inst);
    Assert(module_inst->hw);

    /* Get a pointer to the module's hardware instance */
    tc_registers_t *const tc_module = module_inst->hw;

    tc_module->COUNT32.TC_CTRLBSET = TC_CTRLBSET_CMD_READSYNC;
    while (tc_module->COUNT32.TC_CTRLBSET & TC_CTRLBSET_CMD_Msk);

    /* Read from based on the TC counter size */
    switch (module_inst->counter_size) {
        case TC_COUNTER_SIZE_8BIT:
            return (uint32_t)tc_module->COUNT8.TC_COUNT;

        case TC_COUNTER_SIZE_16BIT:
            return (uint32_t)tc_module->COUNT16.TC_COUNT;

        case TC_COUNTER_SIZE_32BIT:
            return tc_module->COUNT32.TC_COUNT;
    }

    Assert(false);
    return 0;
}

/**
 * \brief Gets the TC module capture value.
 *
 * Retrieves the capture value in the indicated TC module capture channel.
 *
 * \param[in]  module_inst    Pointer to the software module instance struct
 * \param[in]  channel_index  Index of the Compare Capture channel to read
 *
 * \return Capture value stored in the specified timer channel.
 */
uint32_t tc_get_capture_value(
    const struct tc_module *const module_inst,
    const enum tc_compare_capture_channel channel_index)
{
    /* Sanity check arguments */
    Assert(module_inst);
    Assert(module_inst->hw);

    /* Get a pointer to the module's hardware instance */
    tc_registers_t *const tc_module = module_inst->hw;

    tc_module->COUNT32.TC_CTRLBSET = TC_CTRLBSET_CMD_READSYNC;
    while (tc_module->COUNT32.TC_CTRLBSET & TC_CTRLBSET_CMD_Msk);

    /* Read out based on the TC counter size */
    switch (module_inst->counter_size) {
        case TC_COUNTER_SIZE_8BIT:
            if (channel_index <
                    NUMBER_OF_COMPARE_CAPTURE_CHANNELS) {
                return tc_module->COUNT8.TC_CC[channel_index];
            }

        case TC_COUNTER_SIZE_16BIT:
            if (channel_index <
                    NUMBER_OF_COMPARE_CAPTURE_CHANNELS) {
                return tc_module->COUNT16.TC_CC[channel_index];
            }

        case TC_COUNTER_SIZE_32BIT:
            if (channel_index <
                    NUMBER_OF_COMPARE_CAPTURE_CHANNELS) {
                return tc_module->COUNT32.TC_CC[channel_index];
            }
    }

    Assert(false);
    return 0;
}

/**
 * \brief Sets a TC module compare value.
 *
 * Writes a compare value to the given TC module compare/capture channel.
 *
 * \param[in]  module_inst    Pointer to the software module instance struct
 * \param[in]  channel_index  Index of the compare channel to write to
 * \param[in]  compare        New compare value to set
 *
 * \return Status of the compare update procedure.
 *
 * \retval  STATUS_OK               The compare value was updated successfully
 * \retval  STATUS_ERR_INVALID_ARG  An invalid channel index was supplied
 */
enum status_code tc_set_compare_value(
    const struct tc_module *const module_inst,
    const enum tc_compare_capture_channel channel_index,
    const uint32_t compare)
{
    /* Sanity check arguments */
    Assert(module_inst);
    Assert(module_inst->hw);

    /* Get a pointer to the module's hardware instance */
    tc_registers_t *const tc_module = module_inst->hw;

    /* Read out based on the TC counter size */
    switch (module_inst->counter_size) {
        case TC_COUNTER_SIZE_8BIT:
            if (channel_index <
                    NUMBER_OF_COMPARE_CAPTURE_CHANNELS) {
                tc_module->COUNT8.TC_CC[channel_index]  =
                    (uint8_t)compare;
                break;
            }

        case TC_COUNTER_SIZE_16BIT:
            if (channel_index <
                    NUMBER_OF_COMPARE_CAPTURE_CHANNELS) {
                tc_module->COUNT16.TC_CC[channel_index] =
                    (uint16_t)compare;
                break;
            }

        case TC_COUNTER_SIZE_32BIT:
            if (channel_index <
                    NUMBER_OF_COMPARE_CAPTURE_CHANNELS) {
                tc_module->COUNT32.TC_CC[channel_index] =
                    (uint32_t)compare;
                break;
            }

        default:
            return STATUS_ERR_INVALID_ARG;
    }

    while (tc_is_syncing(module_inst)) {
        /* Wait for sync */
    }

    return STATUS_OK;
}

/**
 * \brief Resets the TC module.
 *
 * Resets the TC module, restoring all hardware module registers to their
 * default values and disabling the module. The TC module will not be
 * accessible while the reset is being performed.
 *
 * \note When resetting a 32-bit counter only the master TC module's instance
 *       structure should be passed to the function.
 *
 * \param[in]  module_inst    Pointer to the software module instance struct
 *
 * \return Status of the procedure.
 * \retval STATUS_OK                   The module was reset successfully
 * \retval STATUS_ERR_UNSUPPORTED_DEV  A 32-bit slave TC module was passed to
 *                                     the function. Only use reset on master
 *                                     TC
 */
enum status_code tc_reset(
    const struct tc_module *const module_inst)
{
    /* Sanity check arguments  */
    Assert(module_inst);
    Assert(module_inst->hw);

    /* Get a pointer to the module hardware instance */
    tc_count8_registers_t *const tc_module = &(module_inst->hw->COUNT8);

    if (tc_module->TC_STATUS & TC_STATUS_SLAVE(1)) {
        return STATUS_ERR_UNSUPPORTED_DEV;
    }

    /* Disable this module if it is running */
    if (tc_module->TC_CTRLA & TC_CTRLA_ENABLE(1)) {
        tc_disable(module_inst);
    }

    /* Reset this TC module */
    tc_module->TC_CTRLA  |= TC_CTRLA_SWRST(1);

    return STATUS_OK;
}

/**
 * \brief Set the timer TOP/period value.
 *
 * For 8-bit counter size this function writes the top value to the period
 * register.
 *
 * For 16- and 32-bit counter size this function writes the top value to
 * Capture Compare register 0. The value in this register can not be used for
 * any other purpose.
 *
 * \note This function is designed to be used in PWM or frequency
 *       match modes only. When the counter is set to 16- or 32-bit counter
 *       size. In 8-bit counter size it will always be possible to change the
 *       top value even in normal mode.
 *
 * \param[in]  module_inst   Pointer to the software module instance struct
 * \param[in]  top_value     New timer TOP value to set
 *
 * \return Status of the TOP set procedure.
 *
 * \retval STATUS_OK              The timer TOP value was updated successfully
 * \retval STATUS_ERR_INVALID_ARG The configured TC module counter size in the
 *                                module instance is invalid
 */
enum status_code tc_set_top_value (
    const struct tc_module *const module_inst,
    const uint32_t top_value)
{
    Assert(module_inst);
    Assert(module_inst->hw);
    Assert(top_value);

    tc_registers_t *const tc_module = module_inst->hw;

    switch (module_inst->counter_size) {
        case TC_COUNTER_SIZE_8BIT:
            tc_module->COUNT8.TC_PER    = (uint8_t)top_value;
            break;

        case TC_COUNTER_SIZE_16BIT:
            tc_module->COUNT16.TC_CC[0] = (uint16_t)top_value;
            break;

        case TC_COUNTER_SIZE_32BIT:
            tc_module->COUNT32.TC_CC[0] = (uint32_t)top_value;
            break;

        default:
            Assert(false);
            return STATUS_ERR_INVALID_ARG;
    }

    while (tc_is_syncing(module_inst)) {
        /* Wait for sync */
    }

    return STATUS_OK;
}
