/**
 * \file
 *
 * \brief SAM D21/R21/DA/HA Clock Driver
 *
 * Copyright (C) 2013-2016 Atmel Corporation. All rights reserved.
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
#include "clock.h"
#include "conf_clocks.h"
#include "system.h"

#ifndef SYSCTRL_FUSES_OSC32K_ADDR
#  define SYSCTRL_FUSES_OSC32K_ADDR SYSCTRL_FUSES_OSC32K_CAL_ADDR
#  define SYSCTRL_FUSES_OSC32K_Pos  SYSCTRL_FUSES_OSC32K_CAL_Pos
#endif

/**
 * \internal
 * \brief DFLL-specific data container.
 */
struct _system_clock_dfll_config {
	uint32_t controla;
	uint32_t controlb;
	uint32_t val;
	uint32_t mul;
};

/**
 * \internal
 * \brief DPLL-specific data container.
 */
struct _system_clock_dpll_config {
	uint32_t frequency;
};

/**
 * \internal
 * \brief XOSC-specific data container.
 */
struct _system_clock_xosc_config {
	uint32_t frequency;
};

/**
 * \internal
 * \brief System clock module data container.
 */
struct _system_clock_module {
	volatile struct _system_clock_dfll_config dfll;

#ifdef FEATURE_SYSTEM_CLOCK_DPLL
	volatile struct _system_clock_dpll_config dpll[2];
#endif

	volatile struct _system_clock_xosc_config xosc;
	volatile struct _system_clock_xosc_config xosc32k;
};

/**
 * \internal
 * \brief Internal module instance to cache configuration values.
 */
static struct _system_clock_module _system_clock_inst = {
		.dfll = {
			.controla     = 0,
			.controlb     = 0,
			.val     = 0,
			.mul     = 0,
		},

#ifdef FEATURE_SYSTEM_CLOCK_DPLL
		.dpll[0] = {
			.frequency   = 0,
		},
		.dpll[1] = {
			.frequency   = 0,
		},
#endif
		.xosc = {
			.frequency   = 0,
		},
		.xosc32k = {
			.frequency   = 0,
		},
	};

/**
 * \internal
 * \brief Wait for sync to the DFLL control registers.
 */
static inline void _system_dfll_wait_for_sync(void)
{
	while (!(OSCCTRL_REGS->OSCCTRL_STATUS & OSCCTRL_STATUS_DFLLRDY(1))) {
		/* Wait for DFLL sync */
	}
}

/**
 * \internal
 * \brief Wait for sync to the OSC32K control registers.
 */
static inline void _system_osc32k_wait_for_sync(void)
{
	while (!(OSC32KCTRL_REGS->OSC32KCTRL_STATUS & OSC32KCTRL_STATUS_XOSC32KRDY(1))) {
		/* Wait for OSC32K sync */
	}
}

static inline void _system_clock_source_dfll_set_config(void)
{
	OSCCTRL_REGS->OSCCTRL_DFLLCTRLA = 0;
	OSCCTRL_REGS->OSCCTRL_DFLLMUL = _system_clock_inst.dfll.mul;
	while (OSCCTRL_REGS->OSCCTRL_DFLLSYNC & OSCCTRL_DFLLSYNC_DFLLMUL(1));

	OSCCTRL_REGS->OSCCTRL_DFLLCTRLB = 0;
	while (OSCCTRL_REGS->OSCCTRL_DFLLSYNC & OSCCTRL_DFLLSYNC_DFLLCTRLB(1));

	OSCCTRL_REGS->OSCCTRL_DFLLCTRLA |= OSCCTRL_DFLLCTRLA_ENABLE(1);
	while (OSCCTRL_REGS->OSCCTRL_DFLLSYNC & OSCCTRL_DFLLSYNC_ENABLE(1));

	OSCCTRL_REGS->OSCCTRL_DFLLVAL = _system_clock_inst.dfll.val;
	while (OSCCTRL_REGS->OSCCTRL_DFLLSYNC & OSCCTRL_DFLLSYNC_DFLLVAL(1));

	OSCCTRL_REGS->OSCCTRL_DFLLCTRLB = _system_clock_inst.dfll.controlb;
	while (!(OSCCTRL_REGS->OSCCTRL_STATUS & OSCCTRL_STATUS_DFLLRDY(1)));
}

/**
 * \brief Retrieve the frequency of a clock source.
 *
 * Determines the current operating frequency of a given clock source.
 *
 * \param[in] clock_source  Clock source to get the frequency
 *
 * \returns Frequency of the given clock source, in Hz.
 */
uint32_t system_clock_source_get_hz(
		const enum system_clock_source clock_source)
{
	switch (clock_source) {
	case SYSTEM_CLOCK_SOURCE_ULP32K:
		return 32768UL;

	case SYSTEM_CLOCK_SOURCE_XOSC32K:
		return _system_clock_inst.xosc32k.frequency;

	case SYSTEM_CLOCK_SOURCE_DFLL:

		/* Check if the DFLL has been configured */
		if (!(_system_clock_inst.dfll.controla & OSCCTRL_DFLLCTRLA_ENABLE(1)))
			return 0;

		/* Make sure that the DFLL module is ready */
		_system_dfll_wait_for_sync();

		/* Check if operating in closed loop mode */
		if (_system_clock_inst.dfll.controlb & OSCCTRL_DFLLCTRLB_MODE(1)) {
			return system_gclk_chan_get_hz(GCLK_PCHCTRL_ID_0_GCLK_OSCCTRL_DFLL48) *
					(_system_clock_inst.dfll.mul & 0xffff);
		}

		return 48000000UL;

#ifdef FEATURE_SYSTEM_CLOCK_DPLL
	case SYSTEM_CLOCK_SOURCE_DPLL0:
		if (!(OSCCTRL_REGS->DPLL[0].OSCCTRL_DPLLCTRLA & OSCCTRL_DPLLCTRLA_ENABLE(1))) {
			return 0;
		}

		return _system_clock_inst.dpll[0].frequency;
	case SYSTEM_CLOCK_SOURCE_DPLL1:
		if (!(OSCCTRL_REGS->DPLL[1].OSCCTRL_DPLLCTRLA & OSCCTRL_DPLLCTRLA_ENABLE(1))) {
			return 0;
		}

		return _system_clock_inst.dpll[1].frequency;
#endif

	default:
		return 0;
	}
}

/**
 * \brief Configure the external oscillator clock source.
 *
 * Configures the external oscillator clock source with the given configuration
 * settings.
 *
 * \param[in] config  External oscillator configuration structure containing
 *                    the new config
 */
void system_clock_source_xosc_set_config(
		const uint8_t xosc,
		struct system_clock_source_xosc_config *const config)
{
	uint32_t xoscctrl = OSCCTRL_REGS->OSCCTRL_XOSCCTRL[xosc];
	xoscctrl &= ~(
		OSCCTRL_XOSCCTRL_STARTUP_Msk |
		OSCCTRL_XOSCCTRL_XTALEN_Msk |
		OSCCTRL_XOSCCTRL_LOWBUFGAIN_Msk |
		OSCCTRL_XOSCCTRL_IPTAT_Msk |
		OSCCTRL_XOSCCTRL_IMULT_Msk |
		OSCCTRL_XOSCCTRL_ONDEMAND_Msk |
		OSCCTRL_XOSCCTRL_RUNSTDBY_Msk
	);

	xoscctrl |= OSCCTRL_XOSCCTRL_STARTUP(config->startup_time);

	if (config->external_clock == SYSTEM_CLOCK_EXTERNAL_CRYSTAL) {
		xoscctrl |= OSCCTRL_XOSCCTRL_XTALEN(1);
	}

	xoscctrl |= OSCCTRL_XOSCCTRL_LOWBUFGAIN(config->auto_gain_control);

	/* Set gain if automatic gain control is not selected */
	if (!config->auto_gain_control) {
		if (config->frequency == 8000000) {
			xoscctrl |= OSCCTRL_XOSCCTRL_IMULT(3);
			xoscctrl |= OSCCTRL_XOSCCTRL_IPTAT(2);
		} else if (config->frequency <= 16000000) {
			xoscctrl |= OSCCTRL_XOSCCTRL_IMULT(4);
			xoscctrl |= OSCCTRL_XOSCCTRL_IPTAT(3);
		} else if (config->frequency <= 24000000) {
			xoscctrl |= OSCCTRL_XOSCCTRL_IMULT(5);
			xoscctrl |= OSCCTRL_XOSCCTRL_IPTAT(3);
		} else if (config->frequency <= 48000000) {
			xoscctrl |= OSCCTRL_XOSCCTRL_IMULT(6);
			xoscctrl |= OSCCTRL_XOSCCTRL_IPTAT(3);
		}
	}

	xoscctrl |= OSCCTRL_XOSCCTRL_ONDEMAND(config->on_demand);
	xoscctrl |= OSCCTRL_XOSCCTRL_RUNSTDBY(config->run_in_standby);

	/* Store XOSC frequency for internal use */
	_system_clock_inst.xosc.frequency = config->frequency;

	OSCCTRL_REGS->OSCCTRL_XOSCCTRL[xosc] = xoscctrl;
}

/**
 * \brief Configure the XOSC32K external 32KHz oscillator clock source.
 *
 * Configures the external 32KHz oscillator clock source with the given
 * configuration settings.
 *
 * \param[in] config  XOSC32K configuration structure containing the new config
 */
void system_clock_source_xosc32k_set_config(
		struct system_clock_source_xosc32k_config *const config)
{
	uint32_t xosc32k = OSC32KCTRL_REGS->OSC32KCTRL_XOSC32K;

	xosc32k &= ~(
		OSC32KCTRL_XOSC32K_STARTUP_Msk |
		OSC32KCTRL_XOSC32K_XTALEN_Msk |
		OSC32KCTRL_XOSC32K_CGM_Msk |
		OSC32KCTRL_XOSC32K_EN1K_Msk |
		OSC32KCTRL_XOSC32K_EN32K_Msk |
		OSC32KCTRL_XOSC32K_ONDEMAND_Msk |
		OSC32KCTRL_XOSC32K_RUNSTDBY_Msk |
		OSC32KCTRL_XOSC32K_WRTLOCK_Msk
	);

	xosc32k |= OSC32KCTRL_XOSC32K_STARTUP(config->startup_time);

	if (config->external_clock == SYSTEM_CLOCK_EXTERNAL_CRYSTAL) {
		xosc32k |= OSC32KCTRL_XOSC32K_XTALEN(1);
	}

	xosc32k |= OSC32KCTRL_XOSC32K_CGM_HS;
	xosc32k |= OSC32KCTRL_XOSC32K_EN1K(config->enable_1khz_output);
	xosc32k |= OSC32KCTRL_XOSC32K_EN32K(config->enable_32khz_output);

	xosc32k |= OSC32KCTRL_XOSC32K_ONDEMAND(config->on_demand);
	xosc32k |= OSC32KCTRL_XOSC32K_RUNSTDBY(config->run_in_standby);
	xosc32k |= OSC32KCTRL_XOSC32K_WRTLOCK(config->write_once);

	/* Cache the new frequency in case the user needs to check the current
	 * operating frequency later */
	_system_clock_inst.xosc32k.frequency = config->frequency;

	OSC32KCTRL_REGS->OSC32KCTRL_XOSC32K = xosc32k;
}

/**
 * \brief Configure the DFLL clock source.
 *
 * Configures the Digital Frequency Locked Loop clock source with the given
 * configuration settings.
 *
 * \note The DFLL will be running when this function returns, as the DFLL module
 *       needs to be enabled in order to perform the module configuration.
 *
 * \param[in] config  DFLL configuration structure containing the new config
 */
void system_clock_source_dfll_set_config(
		struct system_clock_source_dfll_config *const config)
{
	_system_clock_inst.dfll.val = OSCCTRL_REGS->OSCCTRL_DFLLVAL;

	_system_clock_inst.dfll.controlb =
			(uint32_t)config->wakeup_lock     |
			(uint32_t)config->stable_tracking |
			(uint32_t)config->quick_lock      |
			(uint32_t)config->chill_cycle;
	_system_clock_inst.dfll.controla =
			OSCCTRL_DFLLCTRLA_ONDEMAND(config->on_demand);

	if (config->loop_mode == SYSTEM_CLOCK_DFLL_LOOP_MODE_CLOSED) {

		_system_clock_inst.dfll.mul =
			OSCCTRL_DFLLMUL_CSTEP(config->coarse_max_step) |
			OSCCTRL_DFLLMUL_FSTEP(config->fine_max_step)   |
			OSCCTRL_DFLLMUL_MUL(config->multiply_factor);

		/* Enable the closed loop mode */
		_system_clock_inst.dfll.controlb |= config->loop_mode;
	}

#ifdef SYSCTRL_DFLLCTRL_USBCRM
	if (config->loop_mode == SYSTEM_CLOCK_DFLL_LOOP_MODE_USB_RECOVERY) {

		_system_clock_inst.dfll.mul =
				OSCCTRL_DFLLMUL_CSTEP(config->coarse_max_step) |
				OSCCTRL_DFLLMUL_FSTEP(config->fine_max_step)   |
				OSCCTRL_DFLLMUL_MUL(config->multiply_factor);

		/* Enable the USB recovery mode */
		_system_clock_inst.dfll.controlb |= config->loop_mode |
				OSCCTRL_DFLLCTRLB_MODE(1) | OSCCTRL_DFLLCTRLB_BPLCKC(1);
	}
#endif
}

#ifdef FEATURE_SYSTEM_CLOCK_DPLL
/**
 * \brief Configure the DPLL clock source.
 *
 * Configures the Digital Phase-Locked Loop clock source with the given
 * configuration settings.
 *
 * \note The DPLL will be running when this function returns, as the DPLL module
 *       needs to be enabled in order to perform the module configuration.
 *
 * \param[in] config  DPLL configuration structure containing the new config
 */
void system_clock_source_dpll_set_config(
		const uint8_t dpll,
		struct system_clock_source_dpll_config *const config)
{
	uint32_t tmpldr;
	uint8_t  tmpldrfrac;
	uint32_t refclk;

	refclk = config->reference_frequency;

	/* Only reference clock REF1 can be divided */
	if ((config->reference_clock == SYSTEM_CLOCK_SOURCE_DPLL_REFERENCE_CLOCK_XOSC0) ||
	    (config->reference_clock == SYSTEM_CLOCK_SOURCE_DPLL_REFERENCE_CLOCK_XOSC1)) {
		refclk = refclk / (2 * (config->reference_divider + 1));
	}

	/* Calculate LDRFRAC and LDR */
	tmpldr = (config->output_frequency << 4) / refclk;
	tmpldrfrac = tmpldr & 0x0f;
	tmpldr = (tmpldr >> 4) - 1;

	OSCCTRL_REGS->DPLL[dpll].OSCCTRL_DPLLCTRLA =
			OSCCTRL_DPLLCTRLA_ONDEMAND(config->on_demand) |
			OSCCTRL_DPLLCTRLA_RUNSTDBY(config->run_in_standby);

	OSCCTRL_REGS->DPLL[dpll].OSCCTRL_DPLLRATIO =
			OSCCTRL_DPLLRATIO_LDRFRAC(tmpldrfrac) |
			OSCCTRL_DPLLRATIO_LDR(tmpldr);
	while (OSCCTRL_REGS->DPLL[dpll].OSCCTRL_DPLLSYNCBUSY & OSCCTRL_DPLLSYNCBUSY_DPLLRATIO(1));

	OSCCTRL_REGS->DPLL[dpll].OSCCTRL_DPLLCTRLB =
			OSCCTRL_DPLLCTRLB_DIV(config->reference_divider) |
			OSCCTRL_DPLLCTRLB_LBYPASS(config->lock_bypass) |
			OSCCTRL_DPLLCTRLB_LTIME(config->lock_time) |
			OSCCTRL_DPLLCTRLB_WUF(config->wake_up_fast) |
			OSCCTRL_DPLLCTRLB_FILTER(config->filter);

	/*
	 * Fck = Fckrx * (LDR + 1 + LDRFRAC / 16)
	 */
	_system_clock_inst.dpll[dpll].frequency =
			(refclk * (((tmpldr + 1) << 4) + tmpldrfrac)) >> 4;
}
#endif

/**
 * \brief Writes the calibration values for a given oscillator clock source.
 *
 * Writes an oscillator calibration value to the given oscillator control
 * registers. The acceptable ranges are:
 *
 * For OSC32K:
 *  - 7 bits (max value 128)
 * For OSC8MHZ:
 *  - 8 bits (Max value 255)
 * For OSCULP:
 *  - 5 bits (Max value 32)
 *
 * \note The frequency range parameter applies only when configuring the 8MHz
 *       oscillator and will be ignored for the other oscillators.
 *
 * \param[in] clock_source       Clock source to calibrate
 * \param[in] calibration_value  Calibration value to write
 * \param[in] freq_range         Frequency range (8MHz oscillator only)
 *
 * \retval STATUS_OK               The calibration value was written
 *                                 successfully.
 * \retval STATUS_ERR_INVALID_ARG  The setting is not valid for selected clock
 *                                 source.
 */
enum status_code system_clock_source_write_calibration(
		const enum system_clock_source clock_source,
		const uint16_t calibration_value,
		const uint8_t freq_range)
{
	switch (clock_source) {
	case SYSTEM_CLOCK_SOURCE_ULP32K:

		if (calibration_value > 32) {
			return STATUS_ERR_INVALID_ARG;
		}

		OSC32KCTRL_REGS->OSC32KCTRL_OSCULP32K &= ~OSC32KCTRL_OSCULP32K_CALIB_Msk;
		OSC32KCTRL_REGS->OSC32KCTRL_OSCULP32K =
				OSC32KCTRL_OSCULP32K_CALIB(calibration_value);
		break;

	default:
		Assert(false);
		return STATUS_ERR_INVALID_ARG;
		break;
	}

	return STATUS_OK;
}

/**
 * \brief Enables a clock source.
 *
 * Enables a clock source which has been previously configured.
 *
 * \param[in] clock_source       Clock source to enable
 *
 * \retval STATUS_OK               Clock source was enabled successfully and
 *                                 is ready
 * \retval STATUS_ERR_INVALID_ARG  The clock source is not available on this
 *                                 device
 */
enum status_code system_clock_source_enable(
		const enum system_clock_source clock_source)
{
	switch (clock_source) {
	case SYSTEM_CLOCK_SOURCE_XOSC0:
		OSCCTRL_REGS->OSCCTRL_XOSCCTRL[0] |= OSCCTRL_XOSCCTRL_ENABLE(1);
		break;
	case SYSTEM_CLOCK_SOURCE_XOSC1:
		OSCCTRL_REGS->OSCCTRL_XOSCCTRL[1] |= OSCCTRL_XOSCCTRL_ENABLE(1);
		break;

	case SYSTEM_CLOCK_SOURCE_XOSC32K:
		OSC32KCTRL_REGS->OSC32KCTRL_XOSC32K |= OSC32KCTRL_XOSC32K_ENABLE(1);
		break;

	case SYSTEM_CLOCK_SOURCE_DFLL:
		_system_clock_inst.dfll.controla |= OSCCTRL_DFLLCTRLA_ENABLE(1);
		_system_clock_source_dfll_set_config();
		break;

#ifdef FEATURE_SYSTEM_CLOCK_DPLL
	case SYSTEM_CLOCK_SOURCE_DPLL0:
		OSCCTRL_REGS->DPLL[0].OSCCTRL_DPLLCTRLA |= OSCCTRL_DPLLCTRLA_ENABLE(1);
		break;
	case SYSTEM_CLOCK_SOURCE_DPLL1:
		OSCCTRL_REGS->DPLL[1].OSCCTRL_DPLLCTRLA |= OSCCTRL_DPLLCTRLA_ENABLE(1);
		break;
#endif

	case SYSTEM_CLOCK_SOURCE_ULP32K:
		/* Always enabled */
		return STATUS_OK;

	default:
		Assert(false);
		return STATUS_ERR_INVALID_ARG;
	}

	return STATUS_OK;
}

/**
 * \brief Disables a clock source.
 *
 * Disables a clock source that was previously enabled.
 *
 * \param[in] clock_source  Clock source to disable
 *
 * \retval STATUS_OK               Clock source was disabled successfully
 * \retval STATUS_ERR_INVALID_ARG  An invalid or unavailable clock source was
 *                                 given
 */
enum status_code system_clock_source_disable(
		const enum system_clock_source clock_source)
{
	switch (clock_source) {
	case SYSTEM_CLOCK_SOURCE_XOSC0:
		OSCCTRL_REGS->OSCCTRL_XOSCCTRL[0] &= ~OSCCTRL_XOSCCTRL_ENABLE_Msk;
		break;
	case SYSTEM_CLOCK_SOURCE_XOSC1:
		OSCCTRL_REGS->OSCCTRL_XOSCCTRL[1] &= ~OSCCTRL_XOSCCTRL_ENABLE_Msk;
		break;

	case SYSTEM_CLOCK_SOURCE_XOSC32K:
		OSC32KCTRL_REGS->OSC32KCTRL_XOSC32K &= ~OSC32KCTRL_XOSC32K_ENABLE_Msk;
		break;

	case SYSTEM_CLOCK_SOURCE_DFLL:
		_system_clock_inst.dfll.controla &= ~OSCCTRL_DFLLCTRLA_ENABLE_Msk;
		OSCCTRL_REGS->OSCCTRL_DFLLCTRLA = _system_clock_inst.dfll.controla;
		break;

#ifdef FEATURE_SYSTEM_CLOCK_DPLL
	case SYSTEM_CLOCK_SOURCE_DPLL0:
		OSCCTRL_REGS->DPLL[0].OSCCTRL_DPLLCTRLA &= ~OSCCTRL_DPLLCTRLA_ENABLE_Msk;
		break;
	case SYSTEM_CLOCK_SOURCE_DPLL1:
		OSCCTRL_REGS->DPLL[1].OSCCTRL_DPLLCTRLA &= ~OSCCTRL_DPLLCTRLA_ENABLE_Msk;
		break;
#endif

	case SYSTEM_CLOCK_SOURCE_ULP32K:
		/* Not possible to disable */

	default:
		Assert(false);
		return STATUS_ERR_INVALID_ARG;
	}

	return STATUS_OK;
}

/**
 * \brief Checks if a clock source is ready.
 *
 * Checks if a given clock source is ready to be used.
 *
 * \param[in] clock_source  Clock source to check if ready
 *
 * \returns Ready state of the given clock source.
 *
 * \retval true   Clock source is enabled and ready
 * \retval false  Clock source is disabled or not yet ready
 */
bool system_clock_source_is_ready(
		const enum system_clock_source clock_source)
{
	switch (clock_source) {
	case SYSTEM_CLOCK_SOURCE_XOSC0:
		return OSCCTRL_REGS->OSCCTRL_STATUS & OSCCTRL_STATUS_XOSCRDY0(1);
	case SYSTEM_CLOCK_SOURCE_XOSC1:
		return OSCCTRL_REGS->OSCCTRL_STATUS & OSCCTRL_STATUS_XOSCRDY1(1);

	case SYSTEM_CLOCK_SOURCE_XOSC32K:
		return OSC32KCTRL_REGS->OSC32KCTRL_STATUS & OSC32KCTRL_STATUS_XOSC32KRDY(1);

	case SYSTEM_CLOCK_SOURCE_DFLL:
		if (CONF_CLOCK_DFLL_LOOP_MODE == SYSTEM_CLOCK_DFLL_LOOP_MODE_CLOSED) {
			if ((OSCCTRL_REGS->OSCCTRL_STATUS & OSCCTRL_STATUS_DFLLRDY(1)) &&
				(OSCCTRL_REGS->OSCCTRL_STATUS & OSCCTRL_STATUS_DFLLLCKF(1)) &&
				(OSCCTRL_REGS->OSCCTRL_STATUS & OSCCTRL_STATUS_DFLLLCKC(1))) {
				return true;
			} else {
				return false;
			}
		} else {
			return OSCCTRL_REGS->OSCCTRL_STATUS & OSCCTRL_STATUS_DFLLRDY(1);
		}

#ifdef FEATURE_SYSTEM_CLOCK_DPLL
	case SYSTEM_CLOCK_SOURCE_DPLL0:
		if ((OSCCTRL_REGS->DPLL[0].OSCCTRL_DPLLSTATUS & OSCCTRL_DPLLSTATUS_CLKRDY(1)) &&
			(OSCCTRL_REGS->DPLL[0].OSCCTRL_DPLLSTATUS & OSCCTRL_DPLLSTATUS_LOCK(1))) {
			return true;
		} else {
			return false;
		}
	case SYSTEM_CLOCK_SOURCE_DPLL1:
		if (
			(OSCCTRL_REGS->DPLL[1].OSCCTRL_DPLLSTATUS & OSCCTRL_DPLLSTATUS_CLKRDY(1)) &&
			(OSCCTRL_REGS->DPLL[1].OSCCTRL_DPLLSTATUS & OSCCTRL_DPLLSTATUS_LOCK(1))) {
			return true;
		} else {
			return false;
		}
#endif

	case SYSTEM_CLOCK_SOURCE_ULP32K:
		/* Not possible to disable */
		return true;

	default:
		return false;
	}
}

/* Include some checks for conf_clocks.h validation */
//#include "clock_config_check.h"

#if !defined(__DOXYGEN__)
/** \internal
 *
 * Configures a Generic Clock Generator with the configuration from \c conf_clocks.h.
 */
#  define _CONF_CLOCK_GCLK_CONFIG(n) \
	if (CONF_CLOCK_GCLK_##n##_ENABLE == true) { \
		struct system_gclk_gen_config gclk_conf;                          \
		system_gclk_gen_get_config_defaults(&gclk_conf);                  \
		gclk_conf.source_clock    = CONF_CLOCK_GCLK_##n##_CLOCK_SOURCE;   \
		gclk_conf.division_factor = CONF_CLOCK_GCLK_##n##_PRESCALER;      \
		gclk_conf.run_in_standby  = CONF_CLOCK_GCLK_##n##_RUN_IN_STANDBY; \
		gclk_conf.output_enable   = CONF_CLOCK_GCLK_##n##_OUTPUT_ENABLE;  \
		system_gclk_gen_set_config(GCLK_GENERATOR_##n, &gclk_conf);       \
		system_gclk_gen_enable(GCLK_GENERATOR_##n);                       \
	}

/** \internal
 *
 * Configures a Generic Clock Generator with the configuration from \c conf_clocks.h,
 * provided that it is not the main Generic Clock Generator channel.
 */
#  define _CONF_CLOCK_GCLK_CONFIG_NONMAIN(n, unused) \
		if (n > 0) { _CONF_CLOCK_GCLK_CONFIG(n, unused); }
#endif

/** \internal
 *
 * Switch all peripheral clock to a not enabled general clock
 * to save power.
 */
static void _switch_peripheral_gclk(void)
{
	uint32_t channel;
	uint32_t nr_channel = 48;
	struct system_gclk_chan_config chan_conf;

#if CONF_CLOCK_GCLK_1_ENABLE == false
	chan_conf.source_generator = GCLK_GENERATOR_1;
#elif CONF_CLOCK_GCLK_2_ENABLE == false
	chan_conf.source_generator = GCLK_GENERATOR_2;
#elif CONF_CLOCK_GCLK_3_ENABLE == false
	chan_conf.source_generator = GCLK_GENERATOR_3;
#elif CONF_CLOCK_GCLK_4_ENABLE == false
	chan_conf.source_generator = GCLK_GENERATOR_4;
#elif CONF_CLOCK_GCLK_5_ENABLE == false
	chan_conf.source_generator = GCLK_GENERATOR_5;
#elif CONF_CLOCK_GCLK_6_ENABLE == false
	chan_conf.source_generator = GCLK_GENERATOR_6;
#elif CONF_CLOCK_GCLK_7_ENABLE == false
	chan_conf.source_generator = GCLK_GENERATOR_7;
#else
	chan_conf.source_generator = GCLK_GENERATOR_7;
#endif

	for (channel = 0; channel < nr_channel; channel++) {
		system_gclk_chan_set_config(channel, &chan_conf);
	}
}

/**
 * \brief Initialize clock system based on the configuration in conf_clocks.h.
 *
 * This function will apply the settings in conf_clocks.h when run from the user
 * application. All clock sources and GCLK generators are running when this function
 * returns.
 *
 * \note OSC8M is always enabled and if user selects other clocks for GCLK generators,
 * the OSC8M default enable can be disabled after system_clock_init. Make sure the
 * clock switch successfully before disabling OSC8M.
 */
void system_clock_init(void)
{
	/* Various bits in the INTFLAG register can be set to one at startup.
	   This will ensure that these bits are cleared */
	SUPC_REGS->SUPC_INTFLAG =
			SUPC_INTFLAG_BOD33RDY(1) |
			SUPC_INTFLAG_BOD33DET(1);

	/* Switch all peripheral clock to a not enabled general clock to save power. */
	_switch_peripheral_gclk();

	/* XOSC */
#if CONF_CLOCK_XOSC0_ENABLE == true
	struct system_clock_source_xosc_config xosc_conf;
	system_clock_source_xosc_get_config_defaults(&xosc_conf);

	xosc_conf.external_clock    = CONF_CLOCK_XOSC0_EXTERNAL_CRYSTAL;
	xosc_conf.startup_time      = CONF_CLOCK_XOSC0_STARTUP_TIME;
	xosc_conf.auto_gain_control = CONF_CLOCK_XOSC0_AUTO_GAIN_CONTROL;
	xosc_conf.frequency         = CONF_CLOCK_XOSC0_EXTERNAL_FREQUENCY;
	xosc_conf.on_demand         = CONF_CLOCK_XOSC0_ON_DEMAND;
	xosc_conf.run_in_standby    = CONF_CLOCK_XOSC0_RUN_IN_STANDBY;

	system_clock_source_xosc_set_config(0, &xosc_conf);
	system_clock_source_enable(SYSTEM_CLOCK_SOURCE_XOSC0);
#endif

#if CONF_CLOCK_XOSC1_ENABLE == true
	struct system_clock_source_xosc_config xosc_conf;
	system_clock_source_xosc_get_config_defaults(&xosc_conf);

	xosc_conf.external_clock    = CONF_CLOCK_XOSC1_EXTERNAL_CRYSTAL;
	xosc_conf.startup_time      = CONF_CLOCK_XOSC1_STARTUP_TIME;
	xosc_conf.auto_gain_control = CONF_CLOCK_XOSC1_AUTO_GAIN_CONTROL;
	xosc_conf.frequency         = CONF_CLOCK_XOSC1_EXTERNAL_FREQUENCY;
	xosc_conf.on_demand         = CONF_CLOCK_XOSC1_ON_DEMAND;
	xosc_conf.run_in_standby    = CONF_CLOCK_XOSC1_RUN_IN_STANDBY;

	system_clock_source_xosc_set_config(1, &xosc_conf);
	system_clock_source_enable(SYSTEM_CLOCK_SOURCE_XOSC1);
#endif

	/* XOSC32K */
#if CONF_CLOCK_XOSC32K_ENABLE == true
	struct system_clock_source_xosc32k_config xosc32k_conf;
	system_clock_source_xosc32k_get_config_defaults(&xosc32k_conf);

	xosc32k_conf.frequency           = 32768UL;
	xosc32k_conf.external_clock      = CONF_CLOCK_XOSC32K_EXTERNAL_CRYSTAL;
	xosc32k_conf.startup_time        = CONF_CLOCK_XOSC32K_STARTUP_TIME;
	xosc32k_conf.auto_gain_control   = CONF_CLOCK_XOSC32K_AUTO_AMPLITUDE_CONTROL;
	xosc32k_conf.enable_1khz_output  = CONF_CLOCK_XOSC32K_ENABLE_1KHZ_OUPUT;
	xosc32k_conf.enable_32khz_output = CONF_CLOCK_XOSC32K_ENABLE_32KHZ_OUTPUT;
	xosc32k_conf.on_demand           = false;
	xosc32k_conf.run_in_standby      = CONF_CLOCK_XOSC32K_RUN_IN_STANDBY;

	system_clock_source_xosc32k_set_config(&xosc32k_conf);
	system_clock_source_enable(SYSTEM_CLOCK_SOURCE_XOSC32K);
	while(!system_clock_source_is_ready(SYSTEM_CLOCK_SOURCE_XOSC32K));
	if (CONF_CLOCK_XOSC32K_ON_DEMAND) {
		OSC32KCTRL_REGS->OSC32KCTRL_XOSC32K |= OSC32KCTRL_XOSC32K_ONDEMAND(1);
	}
#endif

	/* OSCULP32K */
#if CONF_CLOCK_OSC32K_ENABLE == true
	SYSCTRL->OSC32K.bit.CALIB =
			((*(uint32_t *)SYSCTRL_FUSES_OSC32K_ADDR >> 
			SYSCTRL_FUSES_OSC32K_Pos) & 0x7Ful);

	struct system_clock_source_osc32k_config osc32k_conf;
	system_clock_source_osc32k_get_config_defaults(&osc32k_conf);

	osc32k_conf.startup_time        = CONF_CLOCK_OSC32K_STARTUP_TIME;
	osc32k_conf.enable_1khz_output  = CONF_CLOCK_OSC32K_ENABLE_1KHZ_OUTPUT;
	osc32k_conf.enable_32khz_output = CONF_CLOCK_OSC32K_ENABLE_32KHZ_OUTPUT;
	osc32k_conf.on_demand           = CONF_CLOCK_OSC32K_ON_DEMAND;
	osc32k_conf.run_in_standby      = CONF_CLOCK_OSC32K_RUN_IN_STANDBY;

	system_clock_source_osc32k_set_config(&osc32k_conf);
	system_clock_source_enable(SYSTEM_CLOCK_SOURCE_OSC32K);
#endif

	/* DFLL Config (Open and Closed Loop) */
#if CONF_CLOCK_DFLL_ENABLE == true
	struct system_clock_source_dfll_config dfll_conf;
	system_clock_source_dfll_get_config_defaults(&dfll_conf);

	dfll_conf.loop_mode      = CONF_CLOCK_DFLL_LOOP_MODE;
	dfll_conf.on_demand      = false;

#  if CONF_CLOCK_DFLL_QUICK_LOCK == true
	dfll_conf.quick_lock = SYSTEM_CLOCK_DFLL_QUICK_LOCK_ENABLE;
#  else
	dfll_conf.quick_lock = SYSTEM_CLOCK_DFLL_QUICK_LOCK_DISABLE;
#  endif

#  if CONF_CLOCK_DFLL_TRACK_AFTER_FINE_LOCK == true
	dfll_conf.stable_tracking = SYSTEM_CLOCK_DFLL_STABLE_TRACKING_TRACK_AFTER_LOCK;
#  else
	dfll_conf.stable_tracking = SYSTEM_CLOCK_DFLL_STABLE_TRACKING_FIX_AFTER_LOCK;
#  endif

#  if CONF_CLOCK_DFLL_KEEP_LOCK_ON_WAKEUP == true
	dfll_conf.wakeup_lock = SYSTEM_CLOCK_DFLL_WAKEUP_LOCK_KEEP;
#  else
	dfll_conf.wakeup_lock = SYSTEM_CLOCK_DFLL_WAKEUP_LOCK_LOSE;
#  endif

#  if CONF_CLOCK_DFLL_ENABLE_CHILL_CYCLE == true
	dfll_conf.chill_cycle = SYSTEM_CLOCK_DFLL_CHILL_CYCLE_ENABLE;
#  else
	dfll_conf.chill_cycle = SYSTEM_CLOCK_DFLL_CHILL_CYCLE_DISABLE;
#  endif

	if (CONF_CLOCK_DFLL_LOOP_MODE == SYSTEM_CLOCK_DFLL_LOOP_MODE_CLOSED) {
		dfll_conf.multiply_factor = CONF_CLOCK_DFLL_MULTIPLY_FACTOR;
	}

	dfll_conf.coarse_max_step = CONF_CLOCK_DFLL_MAX_COARSE_STEP_SIZE;
	dfll_conf.fine_max_step   = CONF_CLOCK_DFLL_MAX_FINE_STEP_SIZE;

#ifdef SYSCTRL_DFLLCTRL_USBCRM
	if (CONF_CLOCK_DFLL_LOOP_MODE == SYSTEM_CLOCK_DFLL_LOOP_MODE_USB_RECOVERY) {
		dfll_conf.fine_max_step   = 10; 
		dfll_conf.fine_value   = 0x1ff;
		dfll_conf.quick_lock = SYSTEM_CLOCK_DFLL_QUICK_LOCK_ENABLE;
		dfll_conf.stable_tracking = SYSTEM_CLOCK_DFLL_STABLE_TRACKING_TRACK_AFTER_LOCK;
		dfll_conf.wakeup_lock = SYSTEM_CLOCK_DFLL_WAKEUP_LOCK_KEEP;
		dfll_conf.chill_cycle = SYSTEM_CLOCK_DFLL_CHILL_CYCLE_DISABLE;

		dfll_conf.multiply_factor = 48000;
	}
#endif

	system_clock_source_dfll_set_config(&dfll_conf);
#endif

	/* GCLK */
#if CONF_CLOCK_CONFIGURE_GCLK == true
	system_gclk_init();

	if (CONF_CLOCK_CONFIGURE_GCLK == true) {
		struct system_gclk_gen_config gclk_conf;
		system_gclk_gen_get_config_defaults(&gclk_conf);
		gclk_conf.source_clock    = GCLK_GENCTRL_SRC_OSCULP32K;
		gclk_conf.division_factor = 1;
		system_gclk_gen_set_config(0, &gclk_conf);
		system_gclk_gen_enable(0);
	}

	/* Configure all GCLK generators except for the main generator, which
	 * is configured later after all other clock systems are set up */
	_CONF_CLOCK_GCLK_CONFIG(1);
	_CONF_CLOCK_GCLK_CONFIG(3);
	_CONF_CLOCK_GCLK_CONFIG(4);
	_CONF_CLOCK_GCLK_CONFIG(5);
	_CONF_CLOCK_GCLK_CONFIG(6);
	_CONF_CLOCK_GCLK_CONFIG(7);
	_CONF_CLOCK_GCLK_CONFIG(8);
	_CONF_CLOCK_GCLK_CONFIG(9);
	_CONF_CLOCK_GCLK_CONFIG(10);
	_CONF_CLOCK_GCLK_CONFIG(11);

#  if CONF_CLOCK_DFLL_ENABLE == true
	/* Enable DFLL reference clock if in closed loop mode */
	if (CONF_CLOCK_DFLL_LOOP_MODE == SYSTEM_CLOCK_DFLL_LOOP_MODE_CLOSED) {
		struct system_gclk_chan_config dfll_gclk_chan_conf;

		system_gclk_chan_get_config_defaults(&dfll_gclk_chan_conf);
		dfll_gclk_chan_conf.source_generator = CONF_CLOCK_DFLL_SOURCE_GCLK_GENERATOR;
		system_gclk_chan_set_config(GCLK_PCHCTRL_ID_0_GCLK_OSCCTRL_DFLL48,
				&dfll_gclk_chan_conf);
		system_gclk_chan_enable(GCLK_PCHCTRL_ID_0_GCLK_OSCCTRL_DFLL48);
	}
#  endif

#  if CONF_CLOCK_DPLL0_ENABLE == true
	/* Enable DPLL internal lock timer and reference clock */
	struct system_gclk_chan_config dpll0_gclk_chan_conf;
	system_gclk_chan_get_config_defaults(&dpll0_gclk_chan_conf);
	if (CONF_CLOCK_DPLL0_LOCK_TIME != SYSTEM_CLOCK_SOURCE_DPLL_LOCK_TIME_DEFAULT) {
		dpll0_gclk_chan_conf.source_generator = CONF_CLOCK_DPLL0_LOCK_GCLK_GENERATOR;
		system_gclk_chan_set_config(GCLK_PCHCTRL_ID_1_GCLK_OSCCTRL_FDPLL0, &dpll0_gclk_chan_conf);
		system_gclk_chan_enable(GCLK_PCHCTRL_ID_1_GCLK_OSCCTRL_FDPLL0);
	}

	if (CONF_CLOCK_DPLL0_REFERENCE_CLOCK == SYSTEM_CLOCK_SOURCE_DPLL_REFERENCE_CLOCK_GCLK) {
		dpll0_gclk_chan_conf.source_generator = CONF_CLOCK_DPLL0_REFERENCE_GCLK_GENERATOR;
		system_gclk_chan_set_config(GCLK_PCHCTRL_ID_1_GCLK_OSCCTRL_FDPLL0, &dpll0_gclk_chan_conf);
		system_gclk_chan_enable(GCLK_PCHCTRL_ID_1_GCLK_OSCCTRL_FDPLL0);
	}
#  endif

#  if CONF_CLOCK_DPLL1_ENABLE == true
	/* Enable DPLL internal lock timer and reference clock */
	struct system_gclk_chan_config dpll1_gclk_chan_conf;
	system_gclk_chan_get_config_defaults(&dpll1_gclk_chan_conf);
	if (CONF_CLOCK_DPLL1_LOCK_TIME != SYSTEM_CLOCK_SOURCE_DPLL_LOCK_TIME_DEFAULT) {
		dpll1_gclk_chan_conf.source_generator = CONF_CLOCK_DPLL1_LOCK_GCLK_GENERATOR;
		system_gclk_chan_set_config(GCLK_PCHCTRL_ID_2_GCLK_OSCCTRL_FDPLL1, &dpll1_gclk_chan_conf);
		system_gclk_chan_enable(GCLK_PCHCTRL_ID_2_GCLK_OSCCTRL_FDPLL1);
	}

	if (CONF_CLOCK_DPLL1_REFERENCE_CLOCK == SYSTEM_CLOCK_SOURCE_DPLL_REFERENCE_CLOCK_GCLK) {
		dpll1_gclk_chan_conf.source_generator = CONF_CLOCK_DPLL1_REFERENCE_GCLK_GENERATOR;
		system_gclk_chan_set_config(GCLK_PCHCTRL_ID_2_GCLK_OSCCTRL_FDPLL1, &dpll1_gclk_chan_conf);
		system_gclk_chan_enable(GCLK_PCHCTRL_ID_2_GCLK_OSCCTRL_FDPLL1);
	}
#  endif
#endif

	/* DFLL Enable (Open and Closed Loop) */
#if CONF_CLOCK_DFLL_ENABLE == true
	system_clock_source_enable(SYSTEM_CLOCK_SOURCE_DFLL);
	while(!system_clock_source_is_ready(SYSTEM_CLOCK_SOURCE_DFLL));
	if (CONF_CLOCK_DFLL_ON_DEMAND) {
		OSCCTRL_REGS->OSCCTRL_DFLLCTRLA |= OSCCTRL_DFLLCTRLA_ONDEMAND(1);
	}
#endif

	/* DPLL */
#ifdef FEATURE_SYSTEM_CLOCK_DPLL
#  if (CONF_CLOCK_DPLL0_ENABLE == true)
	/* Enable DPLL reference clock */
	if (CONF_CLOCK_DPLL0_REFERENCE_CLOCK == SYSTEM_CLOCK_SOURCE_DPLL_REFERENCE_CLOCK_XOSC32K) {
		/* XOSC32K should have been enabled for DPLL_REF0 */
		Assert(CONF_CLOCK_XOSC32K_ENABLE);
	} else if (CONF_CLOCK_DPLL0_REFERENCE_CLOCK == SYSTEM_CLOCK_SOURCE_DPLL_REFERENCE_CLOCK_XOSC0) {
		/* XOSC should have been enabled for DPLL_REF1 */
		Assert(CONF_CLOCK_XOSC0_ENABLE);
	} else if (CONF_CLOCK_DPLL0_REFERENCE_CLOCK == SYSTEM_CLOCK_SOURCE_DPLL_REFERENCE_CLOCK_XOSC1) {
		/* XOSC should have been enabled for DPLL_REF1 */
		Assert(CONF_CLOCK_XOSC1_ENABLE);
	}
	else if (CONF_CLOCK_DPLL0_REFERENCE_CLOCK == SYSTEM_CLOCK_SOURCE_DPLL_REFERENCE_CLOCK_GCLK) {
		/* GCLK should have been enabled */
		Assert(CONF_CLOCK_CONFIGURE_GCLK);
	}
	else {
		Assert(false);
	}

	struct system_clock_source_dpll_config dpll0_config;
	system_clock_source_dpll_get_config_defaults(&dpll0_config);

	dpll0_config.on_demand        = false;
	dpll0_config.run_in_standby   = CONF_CLOCK_DPLL0_RUN_IN_STANDBY;
	dpll0_config.lock_bypass      = CONF_CLOCK_DPLL0_LOCK_BYPASS;
	dpll0_config.wake_up_fast     = CONF_CLOCK_DPLL0_WAKE_UP_FAST;
	dpll0_config.low_power_enable = CONF_CLOCK_DPLL0_LOW_POWER_ENABLE;

	dpll0_config.filter           = CONF_CLOCK_DPLL0_FILTER;
	dpll0_config.lock_time        = CONF_CLOCK_DPLL0_LOCK_TIME;

	dpll0_config.reference_clock     = CONF_CLOCK_DPLL0_REFERENCE_CLOCK;
	dpll0_config.reference_frequency = CONF_CLOCK_DPLL0_REFERENCE_FREQUENCY;
	dpll0_config.reference_divider   = CONF_CLOCK_DPLL0_REFERENCE_DIVIDER;
	dpll0_config.output_frequency    = CONF_CLOCK_DPLL0_OUTPUT_FREQUENCY;

	system_clock_source_dpll_set_config(0, &dpll0_config);
	system_clock_source_enable(SYSTEM_CLOCK_SOURCE_DPLL0);
	while(!system_clock_source_is_ready(SYSTEM_CLOCK_SOURCE_DPLL0));
	if (CONF_CLOCK_DPLL0_ON_DEMAND) {
		OSCCTRL_REGS->DPLL[0].OSCCTRL_DPLLCTRLA |= OSCCTRL_DPLLCTRLA_ONDEMAND(1);
	}
#  endif

#  if (CONF_CLOCK_DPLL1_ENABLE == true)
	/* Enable DPLL reference clock */
	if (CONF_CLOCK_DPLL1_REFERENCE_CLOCK == SYSTEM_CLOCK_SOURCE_DPLL_REFERENCE_CLOCK_XOSC32K) {
		/* XOSC32K should have been enabled for DPLL_REF0 */
		Assert(CONF_CLOCK_XOSC32K_ENABLE);
	} else if (CONF_CLOCK_DPLL1_REFERENCE_CLOCK == SYSTEM_CLOCK_SOURCE_DPLL_REFERENCE_CLOCK_XOSC0) {
		/* XOSC should have been enabled for DPLL_REF1 */
		Assert(CONF_CLOCK_XOSC0_ENABLE);
	} else if (CONF_CLOCK_DPLL1_REFERENCE_CLOCK == SYSTEM_CLOCK_SOURCE_DPLL_REFERENCE_CLOCK_XOSC1) {
		/* XOSC should have been enabled for DPLL_REF1 */
		Assert(CONF_CLOCK_XOSC1_ENABLE);
	} else if (CONF_CLOCK_DPLL1_REFERENCE_CLOCK == SYSTEM_CLOCK_SOURCE_DPLL_REFERENCE_CLOCK_GCLK) {
		/* GCLK should have been enabled */
		Assert(CONF_CLOCK_CONFIGURE_GCLK);
	}
	else {
		Assert(false);
	}

	struct system_clock_source_dpll_config dpll1_config;
	system_clock_source_dpll_get_config_defaults(&dpll1_config);

	dpll1_config.on_demand        = false;
	dpll1_config.run_in_standby   = CONF_CLOCK_DPLL1_RUN_IN_STANDBY;
	dpll1_config.lock_bypass      = CONF_CLOCK_DPLL1_LOCK_BYPASS;
	dpll1_config.wake_up_fast     = CONF_CLOCK_DPLL1_WAKE_UP_FAST;
	dpll1_config.low_power_enable = CONF_CLOCK_DPLL1_LOW_POWER_ENABLE;

	dpll1_config.filter           = CONF_CLOCK_DPLL1_FILTER;
	dpll1_config.lock_time        = CONF_CLOCK_DPLL1_LOCK_TIME;

	dpll1_config.reference_clock     = CONF_CLOCK_DPLL1_REFERENCE_CLOCK;
	dpll1_config.reference_frequency = CONF_CLOCK_DPLL1_REFERENCE_FREQUENCY;
	dpll1_config.reference_divider   = CONF_CLOCK_DPLL1_REFERENCE_DIVIDER;
	dpll1_config.output_frequency    = CONF_CLOCK_DPLL1_OUTPUT_FREQUENCY;

	system_clock_source_dpll_set_config(1, &dpll1_config);
	system_clock_source_enable(SYSTEM_CLOCK_SOURCE_DPLL1);
	while(!system_clock_source_is_ready(SYSTEM_CLOCK_SOURCE_DPLL1));
	if (CONF_CLOCK_DPLL1_ON_DEMAND) {
		OSCCTRL_REGS->DPLL[1].OSCCTRL_DPLLCTRLA |= OSCCTRL_DPLLCTRLA_ONDEMAND(1);
	}
#  endif
#endif

	/* CPU and BUS clocks */
	system_cpu_clock_set_divider(CONF_CLOCK_CPU_DIVIDER);

	/* GCLK 0 */
#if CONF_CLOCK_CONFIGURE_GCLK == true
	/* Configure the main GCLK last as it might depend on other generators */
	_CONF_CLOCK_GCLK_CONFIG(0);
	_CONF_CLOCK_GCLK_CONFIG(2);
#endif
}
