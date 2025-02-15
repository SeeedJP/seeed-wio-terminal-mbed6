/**
 * \file
 *
 * \brief SAM Generic Clock Driver
 *
 * Copyright (C) 2012-2016 Atmel Corporation. All rights reserved.
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
#ifndef SYSTEM_CLOCK_GCLK_H_INCLUDED
#define SYSTEM_CLOCK_GCLK_H_INCLUDED

/**
 * \addtogroup asfdoc_sam0_system_clock_group
 *
 * @{
 */
#include <cmsis.h>
#include <compiler.h>
//#include "clock_feature.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief List of available GCLK generators.
 *
 * List of Available GCLK generators. This enum is used in the peripheral
 * device drivers to select the GCLK generator to be used for its operation.
 *
 * The number of GCLK generators available is device dependent.
 */
enum gclk_generator {
	/** GCLK generator channel 0 */
	GCLK_GENERATOR_0,
#if defined(__DOXYGEN__) || defined(GCLK_PCHCTRL_GEN_GCLK1)
	/** GCLK generator channel 1 */
	GCLK_GENERATOR_1,
#endif
#if defined(__DOXYGEN__) || defined(GCLK_PCHCTRL_GEN_GCLK2)
	/** GCLK generator channel 2 */
	GCLK_GENERATOR_2,
#endif
#if defined(__DOXYGEN__) || defined(GCLK_PCHCTRL_GEN_GCLK3)
	/** GCLK generator channel 3 */
	GCLK_GENERATOR_3,
#endif
#if defined(__DOXYGEN__) || defined(GCLK_PCHCTRL_GEN_GCLK4)
	/** GCLK generator channel 4 */
	GCLK_GENERATOR_4,
#endif
#if defined(__DOXYGEN__) || defined(GCLK_PCHCTRL_GEN_GCLK5)
	/** GCLK generator channel 5 */
	GCLK_GENERATOR_5,
#endif
#if defined(__DOXYGEN__) || defined(GCLK_PCHCTRL_GEN_GCLK6)
	/** GCLK generator channel 6 */
	GCLK_GENERATOR_6,
#endif
#if defined(__DOXYGEN__) || defined(GCLK_PCHCTRL_GEN_GCLK7)
	/** GCLK generator channel 7 */
	GCLK_GENERATOR_7,
#endif
#if defined(__DOXYGEN__) || defined(GCLK_PCHCTRL_GEN_GCLK8)
	/** GCLK generator channel 8 */
	GCLK_GENERATOR_8,
#endif
#if defined(__DOXYGEN__) || defined(GCLK_PCHCTRL_GEN_GCLK9)
	/** GCLK generator channel 9 */
	GCLK_GENERATOR_9,
#endif
#if defined(__DOXYGEN__) || defined(GCLK_PCHCTRL_GEN_GCLK10)
	/** GCLK generator channel 10 */
	GCLK_GENERATOR_10,
#endif
#if defined(__DOXYGEN__) || defined(GCLK_PCHCTRL_GEN_GCLK11)
	/** GCLK generator channel 11 */
	GCLK_GENERATOR_11,
#endif
#if defined(__DOXYGEN__) || defined(GCLK_PCHCTRL_GEN_GCLK12)
	/** GCLK generator channel 12 */
	GCLK_GENERATOR_12,
#endif
#if defined(__DOXYGEN__) || defined(GCLK_PCHCTRL_GEN_GCLK13)
	/** GCLK generator channel 13 */
	GCLK_GENERATOR_13,
#endif
#if defined(__DOXYGEN__) || defined(GCLK_PCHCTRL_GEN_GCLK14)
	/** GCLK generator channel 14 */
	GCLK_GENERATOR_14,
#endif
#if defined(__DOXYGEN__) || defined(GCLK_PCHCTRL_GEN_GCLK15)
	/** GCLK generator channel 15 */
	GCLK_GENERATOR_15,
#endif
#if defined(__DOXYGEN__) || defined(GCLK_PCHCTRL_GEN_GCLK16)
	/** GCLK generator channel 16 */
	GCLK_GENERATOR_16,
#endif
};

/**
 * \brief Generic Clock Generator configuration structure.
 *
 * Configuration structure for a Generic Clock Generator channel. This
 * structure should be initialized by the
 * \ref system_gclk_gen_get_config_defaults() function before being modified by
 * the user application.
 */
struct system_gclk_gen_config {
	/** Source clock input channel index, see the \ref system_clock_source */
	uint8_t source_clock;
	/** If \c true, the generator output level is high when disabled */
	bool high_when_disabled;
	/** Integer division factor of the clock output compared to the input */
	uint32_t division_factor;
	/** If \c true, the clock is kept enabled during device standby mode */
	bool run_in_standby;
	/** If \c true, enables GCLK generator clock output to a GPIO pin */
	bool output_enable;
};

/**
 * \brief Generic Clock configuration structure.
 *
 * Configuration structure for a Generic Clock channel. This structure
 * should be initialized by the \ref system_gclk_chan_get_config_defaults()
 * function before being modified by the user application.
 */
struct system_gclk_chan_config {
	/** Generic Clock Generator source channel */
	enum gclk_generator source_generator;
};

/** \name Generic Clock Management
 * @{
 */
void system_gclk_init(void);

/** @} */


/**
 * \name Generic Clock Management (Generators)
 * @{
 */
void system_gclk_gen_get_config_defaults(
		struct system_gclk_gen_config *const config);

void system_gclk_gen_set_config(
		const uint8_t generator,
		struct system_gclk_gen_config *const config);

void system_gclk_gen_enable(
		const uint8_t generator);

void system_gclk_gen_disable(
		const uint8_t generator);

bool system_gclk_gen_is_enabled(
		const uint8_t generator);

/** @} */


/**
 * \name Generic Clock Management (Channels)
 * @{
 */

/**
 * \brief Initializes a Generic Clock configuration structure to defaults.
 *
 * Initializes a given Generic Clock configuration structure to a set of
 * known default values. This function should be called on all new
 * instances of these configuration structures before being modified by the
 * user application.
 *
 * The default configuration is as follows:
 *  \li The clock is sourced from the Generic Clock Generator channel 0
 *  \li The clock configuration will not be write-locked when set
 *
 * \param[out] config  Configuration structure to initialize to default values
 */
static inline void system_gclk_chan_get_config_defaults(
		struct system_gclk_chan_config *const config)
{
	/* Sanity check arguments */
	Assert(config);

	/* Default configuration values */
	config->source_generator = GCLK_GENERATOR_0;
}

void system_gclk_chan_set_config(
		const uint8_t channel,
		struct system_gclk_chan_config *const config);

void system_gclk_chan_enable(
		const uint8_t channel);

void system_gclk_chan_disable(
		const uint8_t channel);

bool system_gclk_chan_is_enabled(
		const uint8_t channel);

void system_gclk_chan_lock(
		const uint8_t channel);

bool system_gclk_chan_is_locked(
		const uint8_t channel);

/** @} */


/**
 * \name Generic Clock Frequency Retrieval
 * @{
 */

uint32_t system_gclk_gen_get_hz(
		const uint8_t generator);

uint32_t system_gclk_chan_get_hz(
		const uint8_t channel);

/** @} */

#ifdef __cplusplus
}
#endif

/** @} */

#endif
