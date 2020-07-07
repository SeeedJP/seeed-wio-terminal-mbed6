/**
 * \file
 *
 * \brief SAM Peripheral Analog-to-Digital Converter Driver
 *
 * Copyright (C) 2012-2015 Atmel Corporation. All rights reserved.
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
#include "compiler.h"
#include "adc_drv.h"
#include "PeripheralPins.h"
#include "power.h"

/**
 * \brief Initializes an ADC configuration structure to defaults
 *
 * Initializes a given ADC configuration struct to a set of known default
 * values. This function should be called on any new instance of the
 * configuration struct before being modified by the user application.
 *
 * The default configuration is as follows:
 *  \li GCLK generator 0 (GCLK main) clock source
 *  \li 1V from internal bandgap reference
 *  \li Div 4 clock prescaler
 *  \li 12-bit resolution
 *  \li Window monitor disabled
 *  \li No gain
 *  \li Positive input on ADC PIN 0
 *  \li Negative input on ADC PIN 1
 *  \li Averaging disabled
 *  \li Oversampling disabled
 *  \li Right adjust data
 *  \li Single-ended mode
 *  \li Free running disabled
 *  \li All events (input and generation) disabled
 *  \li Sleep operation disabled
 *  \li No reference compensation
 *  \li No gain/offset correction
 *  \li No added sampling time
 *  \li Pin scan mode disabled
 *
 * \param[out] config  Pointer to configuration struct to initialize to
 *                     default values
 */
void adc_get_config_defaults(struct adc_config *const config)
{
    Assert(config);
    config->clock_source                  = GCLK_GENERATOR_1;
    config->reference                     = ADC_REFERENCE_INTREF;
    config->clock_prescaler               = ADC_CLOCK_PRESCALER_DIV4;
    config->resolution                    = ADC_RESOLUTION_12BIT;
    config->window.window_mode            = ADC_WINDOW_MODE_DISABLE;
    config->window.window_upper_value     = 0;
    config->window.window_lower_value     = 0;
    config->positive_input                = ADC_POSITIVE_INPUT_PIN0;
    config->negative_input                = ADC_NEGATIVE_INPUT_GND;
    config->accumulate_samples            = ADC_ACCUMULATE_DISABLE;
    config->divide_result                 = ADC_DIVIDE_RESULT_DISABLE;
    config->left_adjust                   = false;
    config->differential_mode             = false;
    config->freerunning                   = false;
    config->event_action                  = ADC_EVENT_ACTION_DISABLED;
    config->run_in_standby                = false;
    config->reference_compensation_enable = false;
    config->correction.correction_enable  = false;
    config->correction.gain_correction    = ADC_GAINCORR_RESETVALUE;
    config->correction.offset_correction  = ADC_OFFSETCORR_RESETVALUE;
    config->sample_length                 = 0;
    config->pin_scan.offset_start_scan    = 0;
    config->pin_scan.inputs_to_scan       = 0;
}

/**
 * \brief Sets the ADC window mode
 *
 * Sets the ADC window mode to a given mode and value range.
 *
 * \param[in] module_inst         Pointer to the ADC software instance struct
 * \param[in] window_mode         Window monitor mode to set
 * \param[in] window_lower_value  Lower window monitor threshold value
 * \param[in] window_upper_value  Upper window monitor threshold value
  */
void adc_set_window_mode(
    struct adc_module *const module_inst,
    const enum adc_window_mode window_mode,
    const int16_t window_lower_value,
    const int16_t window_upper_value)
{
    /* Sanity check arguments */
    Assert(module_inst);
    Assert(module_inst->hw);

    adc_registers_t *const adc_module = module_inst->hw;

    while (adc_is_syncing(module_inst)) {
        /* Wait for synchronization */
    }

    /* Set window mode */
    adc_module->ADC_CTRLB = ADC_CTRLB_WINMODE(window_mode);

    while (adc_is_syncing(module_inst)) {
        /* Wait for synchronization */
    }

    /* Set lower window monitor threshold value */
    adc_module->ADC_WINLT = ADC_WINLT_WINLT(window_lower_value);

    while (adc_is_syncing(module_inst)) {
        /* Wait for synchronization */
    }

    /* Set upper window monitor threshold value */
    adc_module->ADC_WINUT = ADC_WINUT_WINUT(window_upper_value);
}

/**
* \internal Configure MUX settings for the analog pins
*
* This function will set the given ADC input pins
* to the analog function in the pinmux, giving
* the ADC access to the analog signal
*
* \param [in] pin AINxx pin to configure
*/
static inline void _adc_configure_ain_pin(adc_registers_t *const adc_module, uint32_t ain)
{
    PinName pin = NC;
    for (int i = 0; PinMap_ADC[i].pin != NC; i++) {
        if (((PinMap_ADC[i].peripheral & ~0xf) == (int)adc_module) &&
            (((PinMap_ADC[i].function >> 4) & 0xf) == ain)) {
                pin = PinMap_ADC[i].pin;
        }
    }
    if (pin == NC) {
        return;
    }

    struct system_pinmux_config config;
    system_pinmux_get_config_defaults(&config);

    /* Analog functions are all on MUX setting B */
    config.input_pull   = SYSTEM_PINMUX_PIN_PULL_NONE;
    config.mux_position = 1;

    system_pinmux_pin_set_config(pin, &config);
}

/**
 * \internal Writes an ADC configuration to the hardware module
 *
 * Writes out a given ADC module configuration to the hardware module.
 *
 * \param[out] module_inst  Pointer to the ADC software instance struct
 * \param[in]  config       Pointer to configuration struct
 *
 * \return Status of the configuration procedure
 * \retval STATUS_OK               The configuration was successful
 * \retval STATUS_ERR_INVALID_ARG  Invalid argument(s) were provided
 */
static enum status_code _adc_set_config(
    struct adc_module *const module_inst,
    struct adc_config *const config)
{
    uint8_t adjres = 0;
    uint32_t resolution = ADC_RESOLUTION_16BIT;
    enum adc_accumulate_samples accumulate = ADC_ACCUMULATE_DISABLE;

    /* Get the hardware module pointer */
    adc_registers_t *const adc_module = module_inst->hw;

    uint32_t chan_index;
    if (adc_module == ADC0_REGS) {
        chan_index = GCLK_PCHCTRL_ID_40_GCLK_ADC0;
    } else if (adc_module == ADC1_REGS) {
        chan_index = GCLK_PCHCTRL_ID_41_GCLK_ADC1;
    } else {
        return STATUS_ERR_INVALID_ARG;
    }

    /* Configure GCLK channel and enable clock */
    struct system_gclk_chan_config gclk_chan_conf;
    system_gclk_chan_get_config_defaults(&gclk_chan_conf);
    gclk_chan_conf.source_generator = config->clock_source;
    system_gclk_chan_set_config(chan_index, &gclk_chan_conf);
    system_gclk_chan_enable(chan_index);

    /* Setup pinmuxing for analog inputs */
    if (config->pin_scan.inputs_to_scan != 0) {
        uint8_t offset = config->pin_scan.offset_start_scan;
        uint8_t start_pin =
            offset +(uint8_t)config->positive_input;
        uint8_t end_pin =
            start_pin + config->pin_scan.inputs_to_scan;

        while (start_pin < end_pin) {
            _adc_configure_ain_pin(adc_module, (offset % 16)+(uint8_t)config->positive_input);
            start_pin++;
            offset++;
        }
        _adc_configure_ain_pin(adc_module, config->negative_input);
    } else {
        _adc_configure_ain_pin(adc_module, config->positive_input);
        _adc_configure_ain_pin(adc_module, config->negative_input);
    }

    /* Configure run in standby */
    adc_module->ADC_CTRLA = (config->run_in_standby << ADC_CTRLA_RUNSTDBY_Pos);

    /* Configure reference */
    adc_module->ADC_REFCTRL =
        (config->reference_compensation_enable << ADC_REFCTRL_REFCOMP_Pos) |
        (config->reference);

    /* Set adjusting result and number of samples */
    switch (config->resolution) {

        case ADC_RESOLUTION_CUSTOM:
            adjres = config->divide_result;
            accumulate = config->accumulate_samples;
            /* 16-bit result register */
            resolution = ADC_RESOLUTION_16BIT;
            break;

        case ADC_RESOLUTION_13BIT:
            /* Increase resolution by 1 bit */
            adjres = ADC_DIVIDE_RESULT_2;
            accumulate = ADC_ACCUMULATE_SAMPLES_4;
            /* 16-bit result register */
            resolution = ADC_RESOLUTION_16BIT;
            break;

        case ADC_RESOLUTION_14BIT:
            /* Increase resolution by 2 bit */
            adjres = ADC_DIVIDE_RESULT_4;
            accumulate = ADC_ACCUMULATE_SAMPLES_16;
            /* 16-bit result register */
            resolution = ADC_RESOLUTION_16BIT;
            break;
        case ADC_RESOLUTION_15BIT:
            /* Increase resolution by 3 bit */
            adjres = ADC_DIVIDE_RESULT_2;
            accumulate = ADC_ACCUMULATE_SAMPLES_64;
            /* 16-bit result register */
            resolution = ADC_RESOLUTION_16BIT;
            break;

        case ADC_RESOLUTION_16BIT:
            /* Increase resolution by 4 bit */
            adjres = ADC_DIVIDE_RESULT_DISABLE;
            accumulate = ADC_ACCUMULATE_SAMPLES_256;
            /* 16-bit result register */
            resolution = ADC_RESOLUTION_16BIT;
            break;
        case ADC_RESOLUTION_8BIT:
            /* 8-bit result register */
            resolution = ADC_RESOLUTION_8BIT;
            break;
        case ADC_RESOLUTION_10BIT:
            /* 10-bit result register */
            resolution = ADC_RESOLUTION_10BIT;
            break;
        case ADC_RESOLUTION_12BIT:
            /* 12-bit result register */
            resolution = ADC_RESOLUTION_12BIT;
            break;

        default:
            /* Unknown. Abort. */
            return STATUS_ERR_INVALID_ARG;
    }

    adc_module->ADC_AVGCTRL = ADC_AVGCTRL_ADJRES(adjres) | accumulate;

    /* Check validity of sample length value */
    if (config->sample_length > 63) {
        return STATUS_ERR_INVALID_ARG;
    } else {
        /* Configure sample length */
        adc_module->ADC_SAMPCTRL =
            (config->sample_length << ADC_SAMPCTRL_SAMPLEN_Pos);
    }

    while (adc_is_syncing(module_inst)) {
        /* Wait for synchronization */
    }

    /* Configure CTRLB */
    adc_module->ADC_CTRLB =
        config->clock_prescaler |
        resolution |
        (config->correction.correction_enable << ADC_CTRLB_CORREN_Pos) |
        (config->freerunning << ADC_CTRLB_FREERUN_Pos) |
        (config->left_adjust << ADC_CTRLB_LEFTADJ_Pos);

    /* Check validity of window thresholds */
    if (config->window.window_mode != ADC_WINDOW_MODE_DISABLE) {
        switch (resolution) {
            case ADC_RESOLUTION_8BIT:
                if (config->differential_mode &&
                        (config->window.window_lower_value > 127 ||
                         config->window.window_lower_value < -128 ||
                         config->window.window_upper_value > 127 ||
                         config->window.window_upper_value < -128)) {
                    /* Invalid value */
                    return STATUS_ERR_INVALID_ARG;
                } else if (config->window.window_lower_value > 255 ||
                           config->window.window_upper_value > 255) {
                    /* Invalid value */
                    return STATUS_ERR_INVALID_ARG;
                }
                break;
            case ADC_RESOLUTION_10BIT:
                if (config->differential_mode &&
                        (config->window.window_lower_value > 511 ||
                         config->window.window_lower_value < -512 ||
                         config->window.window_upper_value > 511 ||
                         config->window.window_upper_value < -512)) {
                    /* Invalid value */
                    return STATUS_ERR_INVALID_ARG;
                } else if (config->window.window_lower_value > 1023 ||
                           config->window.window_upper_value > 1023) {
                    /* Invalid value */
                    return STATUS_ERR_INVALID_ARG;
                }
                break;
            case ADC_RESOLUTION_12BIT:
                if (config->differential_mode &&
                        (config->window.window_lower_value > 2047 ||
                         config->window.window_lower_value < -2048 ||
                         config->window.window_upper_value > 2047 ||
                         config->window.window_upper_value < -2048)) {
                    /* Invalid value */
                    return STATUS_ERR_INVALID_ARG;
                } else if (config->window.window_lower_value > 4095 ||
                           config->window.window_upper_value > 4095) {
                    /* Invalid value */
                    return STATUS_ERR_INVALID_ARG;
                }
                break;
            case ADC_RESOLUTION_16BIT:
                if (config->differential_mode &&
                        (config->window.window_lower_value > 32767 ||
                         config->window.window_lower_value < -32768 ||
                         config->window.window_upper_value > 32767 ||
                         config->window.window_upper_value < -32768)) {
                    /* Invalid value */
                    return STATUS_ERR_INVALID_ARG;
                } else if (config->window.window_lower_value > 65535 ||
                           config->window.window_upper_value > 65535) {
                    /* Invalid value */
                    return STATUS_ERR_INVALID_ARG;
                }
                break;
        }
    }

    while (adc_is_syncing(module_inst)) {
        /* Wait for synchronization */
    }

    /* Configure window mode */
    adc_module->ADC_CTRLB =
        (adc_module->ADC_CTRLB & ~ADC_CTRLB_WINMODE_Msk) |
        ADC_CTRLB_WINMODE(config->window.window_mode);

    while (adc_is_syncing(module_inst)) {
        /* Wait for synchronization */
    }

    /* Configure lower threshold */
    adc_module->ADC_WINLT = ADC_WINLT_WINLT(config->window.window_lower_value);

    while (adc_is_syncing(module_inst)) {
        /* Wait for synchronization */
    }

    /* Configure lower threshold */
    adc_module->ADC_WINUT = ADC_WINUT_WINUT(config->window.window_upper_value);

    /* Configure pin scan mode and positive and negative input pins */
    adc_module->ADC_INPUTCTRL =
        config->negative_input |
        config->positive_input |
        (config->differential_mode << ADC_INPUTCTRL_DIFFMODE_Pos);

    /* Configure events */
    adc_module->ADC_EVCTRL = config->event_action;

    /* Disable all interrupts */
    adc_module->ADC_INTENCLR = (1 << ADC_INTENCLR_WINMON_Pos) |
        (1 << ADC_INTENCLR_OVERRUN_Pos) | (1 << ADC_INTENCLR_RESRDY_Pos);

    if (config->correction.correction_enable) {
        /* Make sure gain_correction value is valid */
        if (config->correction.gain_correction > ADC_GAINCORR_GAINCORR_Msk) {
            return STATUS_ERR_INVALID_ARG;
        } else {
            /* Set gain correction value */
            adc_module->ADC_GAINCORR = config->correction.gain_correction <<
                                       ADC_GAINCORR_GAINCORR_Pos;
        }

        /* Make sure offset correction value is valid */
        if (config->correction.offset_correction > 2047 ||
                config->correction.offset_correction < -2048) {
            return STATUS_ERR_INVALID_ARG;
        } else {
            /* Set offset correction value */
            adc_module->ADC_OFFSETCORR = config->correction.offset_correction <<
                                         ADC_OFFSETCORR_OFFSETCORR_Pos;
        }
    }

    uint32_t biascomp, biasr2r, biasref;
    if (adc_module == ADC0_REGS) {
        biascomp = ((*(uint32_t *)SW0_ADDR) >> 2) & 0x7;
        biasr2r = ((*(uint32_t *)SW0_ADDR) >> 8) & 0x7;
        biasref = ((*(uint32_t *)SW0_ADDR) >> 5) & 0x7;
    } else if (adc_module == ADC1_REGS) {
        biascomp = ((*(uint32_t *)SW0_ADDR) >> 16) & 0x7;
        biasr2r = ((*(uint32_t *)SW0_ADDR) >> 22) & 0x7;
        biasref = ((*(uint32_t *)SW0_ADDR) >> 19) & 0x7;
    }

    /* Load in the fixed device ADC calibration constants */
    adc_module->ADC_CALIB =
        ADC_CALIB_BIASCOMP(biascomp) | ADC_CALIB_BIASR2R(biasr2r) | ADC_CALIB_BIASREFBUF(biasref);

    return STATUS_OK;
}

/**
 * \brief Initializes the ADC
 *
 * Initializes the ADC device struct and the hardware module based on the
 * given configuration struct values.
 *
 * \param[out] module_inst Pointer to the ADC software instance struct
 * \param[in]  hw          Pointer to the ADC module instance
 * \param[in]  config      Pointer to the configuration struct
 *
 * \return Status of the initialization procedure.
 * \retval STATUS_OK                The initialization was successful
 * \retval STATUS_ERR_INVALID_ARG   Invalid argument(s) were provided
 * \retval STATUS_BUSY          The module is busy with a reset operation
 * \retval STATUS_ERR_DENIED        The module is enabled
 */
enum status_code adc_init(
    struct adc_module *const module_inst,
    adc_registers_t *hw,
    struct adc_config *config)
{
    /* Sanity check arguments */
    Assert(module_inst);
    Assert(hw);
    Assert(config);

    /* Associate the software module instance with the hardware module */
    module_inst->hw = hw;

    uint32_t apbmask;
    if (module_inst->hw == ADC0_REGS) {
        apbmask = MCLK_APBDMASK_ADC0(1);
    } else if (module_inst->hw == ADC1_REGS) {
        apbmask = MCLK_APBDMASK_ADC1(1);
    } else {
        return STATUS_ERR_INVALID_ARG;
    }
    /* Turn on the digital interface clock */
    system_apb_clock_set_mask(SYSTEM_CLOCK_APB_APBD, apbmask);

    if (hw->ADC_CTRLA & ADC_CTRLA_SWRST(1)) {
        /* We are in the middle of a reset. Abort. */
        return STATUS_BUSY;
    }

    if (hw->ADC_CTRLA & ADC_CTRLA_ENABLE(1)) {
        /* Module must be disabled before initialization. Abort. */
        return STATUS_ERR_DENIED;
    }

    /* Store the selected reference for later use */
    module_inst->reference = config->reference;

    /* Make sure bandgap is enabled if requested by the config */
    if (module_inst->reference == ADC_REFERENCE_INTREF) {
        system_voltage_reference_enable(SYSTEM_VOLTAGE_REFERENCE_BANDGAP);
    }

#if ADC_CALLBACK_MODE == true
    for (uint8_t i = 0; i < ADC_CALLBACK_N; i++) {
        module_inst->callback[i] = NULL;
    };

    module_inst->registered_callback_mask = 0;
    module_inst->enabled_callback_mask = 0;
    module_inst->remaining_conversions = 0;
    module_inst->job_status = STATUS_OK;

    _adc_instances[0] = module_inst;

    if (config->event_action == ADC_EVENT_ACTION_DISABLED &&
            !config->freerunning) {
        module_inst->software_trigger = true;
    } else {
        module_inst->software_trigger = false;
    }
#endif

    /* Write configuration to module */
    return _adc_set_config(module_inst, config);
}
