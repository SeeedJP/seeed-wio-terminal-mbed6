/* mbed Microcontroller Library
 * Copyright (c) 2006-2015 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <stddef.h>
#include "us_ticker_api.h"
#include "cmsis.h"
#include "mbed_assert.h"
#include "compiler.h"
#include "system.h"
#include "tc_drv.h"
#include "tc_interrupt.h"

#define TICKER_COUNTER_uS		TC2_REGS
#define TICKER_COUNTER_IRQn		TC2_IRQn
#define TICKER_COUNTER_Handlr	TC2_Handler
#define TICKER_COUNTER_BIT_WIDTH TC_COUNTER_SIZE_16BIT

static int us_ticker_inited = 0;
extern uint8_t g_sys_init;

struct tc_module us_ticker_module;

void us_ticker_irq_handler_internal(struct tc_module* us_tc_module)
{
    uint32_t status_flags;

    /* Clear TC capture overflow and TC count overflow */
    status_flags = TC_STATUS_CAPTURE_OVERFLOW | TC_STATUS_COUNT_OVERFLOW;
    tc_clear_status(&us_ticker_module, status_flags);

    us_ticker_irq_handler();
}

void us_ticker_init(void)
{
    struct tc_config config_tc;

    if (us_ticker_inited) return;
    us_ticker_inited = 1;

    if (g_sys_init == 0) {
        system_init();
        g_sys_init = 1;
    }

    tc_get_config_defaults(&config_tc);
    config_tc.clock_source = GCLK_GENERATOR_5;
    config_tc.clock_prescaler = TC_CLOCK_PRESCALER_DIV1;
    config_tc.counter_size = TICKER_COUNTER_BIT_WIDTH;
    config_tc.run_in_standby = true;
    config_tc.wave_generation = TC_WAVE_GENERATION_NORMAL_FREQ_MODE;

    /* Initialize the timer */
    tc_init(&us_ticker_module, TICKER_COUNTER_uS, &config_tc);

    /* Register callback function */
    tc_register_callback(&us_ticker_module, (tc_callback_t)us_ticker_irq_handler_internal, TC_CALLBACK_CC_CHANNEL0);

    TICKER_COUNTER_uS->COUNT32.TC_INTFLAG = TC_INTFLAG_Msk;
    TICKER_COUNTER_uS->COUNT32.TC_INTENCLR = TC_INTENCLR_Msk;

    /* Enable the timer module */
    tc_enable(&us_ticker_module);

    /* NVIC_DisableIRQ(TICKER_COUNTER_IRQn); */
    /* NVIC_SetVector(TICKER_COUNTER_IRQn, (uint32_t)TICKER_COUNTER_Handlr); */
    NVIC_EnableIRQ(TICKER_COUNTER_IRQn);
}

uint32_t us_ticker_read(void)
{
    return tc_get_count_value(&us_ticker_module);
}

void us_ticker_set_interrupt(timestamp_t timestamp)
{
    /* Enable the callback */
    tc_set_compare_value(&us_ticker_module, TC_COMPARE_CAPTURE_CHANNEL_0, (uint32_t)timestamp);
    tc_enable_callback(&us_ticker_module, TC_CALLBACK_CC_CHANNEL0);
}

void us_ticker_disable_interrupt(void)
{
    /* Disable the callback */
    tc_disable_callback(&us_ticker_module, TC_CALLBACK_CC_CHANNEL0);
}

void us_ticker_clear_interrupt(void)
{
    uint32_t status_flags;

    /* Clear TC channel 0 match */
    status_flags = TC_STATUS_CHANNEL_0_MATCH;
    tc_clear_status(&us_ticker_module, status_flags);
}

void us_ticker_free(void)
{
    tc_disable_callback(&us_ticker_module, TC_CALLBACK_CC_CHANNEL0);

    NVIC_DisableIRQ(TICKER_COUNTER_IRQn);

    tc_disable(&us_ticker_module);
}

void us_ticker_fire_interrupt(void)
{
    us_ticker_irq_handler();
}

const ticker_info_t *us_ticker_get_info(void)
{
    static const ticker_info_t info = {
        1000000,
        16
    };
    return &info;
}