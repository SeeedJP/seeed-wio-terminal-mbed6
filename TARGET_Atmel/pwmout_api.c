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
#include "mbed_assert.h"
#include "pwmout_api.h"

#include "cmsis.h"
#include "tcc_drv.h"
#include "tc_drv.h"

#include "pinmap_function.h"

/* Prescaler values for TC,TCC Module */
const uint32_t tc_tcc_prescaler[] = {
    0, /* TC(C)_CLOCK_PRESCALER_DIV1 */
    1, /* TC(C)_CLOCK_PRESCALER_DIV2 */
    2, /* TC(C)_CLOCK_PRESCALER_DIV4 */
    3, /* TC(C)_CLOCK_PRESCALER_DIV8 */
    4, /* TC(C)_CLOCK_PRESCALER_DIV16 */
    6, /* TC(C)_CLOCK_PRESCALER_DIV64 */
    8, /* TC(C)_CLOCK_PRESCALER_DIV256 */
    10, /* TC(C)_CLOCK_PRESCALER_DIV1024 */
};

/** Set the period of PWM object (will not update the waveform)
 *
 * @param[in] obj        The PWM object whose period is to be updated
 * @param[in] period_us  Period in microseconds
 * @return    void
 */
static void pwmout_set_period(pwmout_t* obj, int period_us)
{
    uint32_t freq_hz;
    uint32_t div_freq;
    double us_per_cycle;
    uint64_t max_period = 0;
    uint32_t us_period = period_us;

    /* Sanity check arguments */
    MBED_ASSERT(obj);

    uint32_t count_max = (1 << obj->bit_width) - 1;

    freq_hz = system_gclk_gen_get_hz(obj->clock_source);

    for (int i = 0; i < sizeof(tc_tcc_prescaler) / sizeof(tc_tcc_prescaler[0]); i++) {
        div_freq = freq_hz >> tc_tcc_prescaler[i];
        if (!div_freq) {
            break;
        }
        us_per_cycle = 1000000.00 / div_freq;
        max_period = us_per_cycle * count_max;
        if (max_period >= us_period) {
            obj->clock_prescaler = i;
            obj->period = us_period / us_per_cycle;
            obj->us_per_cycle = us_per_cycle;
            break;
        }
    }
}

static bool is_tcc(pwmout_t* obj)
{
    uint32_t peripheral = pinmap_peripheral(obj->pin, PinMap_PWM);
    switch (peripheral) {
        case TCC0_BASE_ADDRESS:
        case TCC1_BASE_ADDRESS:
        case TCC2_BASE_ADDRESS:
        case TCC3_BASE_ADDRESS:
        case TCC4_BASE_ADDRESS:
            return true;
        default:
            return false;
    }
}

/** Initialize PWM Module with updated values
 *
 * @param[in][out] obj  The PWM object to initialize
 * @return         non-zero if success
 */
static bool pwmout_init_hw_tcc(pwmout_t* obj)
{
    uint32_t mux_func = (uint32_t)NC;
    uint32_t pwm = (uint32_t)NC;
    PinName pin;
    uint32_t ch_index = (uint32_t)NC;
    struct tcc_config config_tcc;
    uint32_t tcc_channel = (uint32_t)NC;

    /* Sanity check arguments */
    MBED_ASSERT(obj);

    pin = obj->pin;
    pwm = pinmap_peripheral(pin, PinMap_PWM);
    if (pwm == (uint32_t)NC) {
        return 0; /* Pin not supported */
    }

    mux_func = pinmap_function(pin, PinMap_PWM) & 0xf;
    ch_index = pinmap_channel_pwm(pin, (PWMName) pwm);
    if ((mux_func == (uint32_t)NC) || (ch_index == (uint32_t)NC)) {
        /* Pin not supported */
        return 0;
    }

    tcc_channel = ch_index;
    if (ch_index == 6 || ch_index == 7) {
        tcc_channel = ch_index - 4;
    };

    tcc_get_config_defaults(&config_tcc, (tcc_registers_t*)pwm);

    config_tcc.counter.clock_source = obj->clock_source;
    config_tcc.counter.clock_prescaler = (enum tcc_clock_prescaler)obj->clock_prescaler;

    config_tcc.counter.period = obj->period;
    config_tcc.compare.wave_generation = TCC_WAVE_GENERATION_SINGLE_SLOPE_PWM;
    config_tcc.compare.match[tcc_channel] = obj->period * obj->duty_cycle;
    config_tcc.pins.enable_wave_out_pin[ch_index] = true;
    config_tcc.pins.wave_out_pin[ch_index]        = pin;
    config_tcc.pins.wave_out_pin_mux[ch_index]    = mux_func;

    return (STATUS_OK == tcc_init(&obj->module.tcc, (tcc_registers_t*)pwm, &config_tcc));
}

static bool pwmout_init_hw_tc(pwmout_t* obj)
{
    struct tc_config config_tc;
    tc_get_config_defaults(&config_tc);
    config_tc.clock_source = obj->clock_source;
    config_tc.clock_prescaler = obj->clock_prescaler;
    config_tc.counter_size = TC_COUNTER_SIZE_16BIT;
    config_tc.run_in_standby = true;
    config_tc.wave_generation = TC_WAVE_GENERATION_MATCH_PWM_MODE;

    config_tc.pwm_channel[1].enabled = true;
    config_tc.pwm_channel[1].pin_mux = pinmap_function(obj->pin, PinMap_PWM) & 0xf;
    config_tc.pwm_channel[1].pin_out = obj->pin;
    config_tc.counter_16_bit.value = 0;
    config_tc.counter_16_bit.compare_capture_channel[0] = obj->period;
    config_tc.counter_16_bit.compare_capture_channel[1] = (uint16_t)(obj->period * obj->duty_cycle);

    /* Initialize the timer */
    uint32_t peripheral = pinmap_peripheral(obj->pin, PinMap_PWM);
    return (STATUS_OK == tc_init(&obj->module.tc, (tc_registers_t *)peripheral, &config_tc));
}

static bool pwmout_init_hw(pwmout_t* obj)
{
    if (is_tcc(obj)) {
        return pwmout_init_hw_tcc(obj);
    } else {
        return pwmout_init_hw_tc(obj);
    }
}

static void pwmout_enable(pwmout_t* obj)
{
    if (is_tcc(obj)) {
        tcc_enable(&obj->module.tcc);
    } else {
        tc_enable(&obj->module.tc);
    }
}

static void pwmout_disable(pwmout_t* obj)
{
    if (is_tcc(obj)) {
        tcc_disable(&obj->module.tcc);
    } else {
        tc_disable(&obj->module.tc);
    }
}

/** Initialize PWM Module
 *
 * @param[in][out] obj  The PWM object to initialize
 * @return         void
 */
void pwmout_init(pwmout_t* obj, PinName pin)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);

    if ((uint32_t)NC == pinmap_peripheral(pin, PinMap_PWM)) {
        /* Pin not supported */
        return;
    }

    obj->pin = pin;
    if (is_tcc(obj)) {
        obj->bit_width = 24;
    } else {
        obj->bit_width = 16;
    }

    /* default setting: period = 100us, duty = 0%, clock = 16MHz */
    obj->clock_source = GCLK_GENERATOR_0;
    obj->duty_cycle = 0.0;
    pwmout_set_period(obj, 100);

    /* Update the changes */
    if (pwmout_init_hw(obj)) {
        /* Enable PWM Module */
        pwmout_enable(obj);
    }
}

/** Free the PWM Module
 *
 * @param[in] obj  The PWM object to free
 * @return    void
 */
void pwmout_free(pwmout_t* obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);

    pwmout_disable(obj);
}

/** Set the duty cycle of PWM Waveform
 *
 * @param[in] obj    The PWM object
 * @param[in] value  New duty cycle to be set
 * @return    void
 */
void pwmout_write(pwmout_t* obj, float value)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);

    if (value < 0.0f) {
        value = 0.0f;
    } else if (value > 1.0f) {
        value = 1.0f;
    }

    /* Modify the pulse width keeping period same */
    obj->duty_cycle = value;

    /* Disable PWM Module */
    pwmout_disable(obj);

    /* Update the changes */
    if (pwmout_init_hw(obj)) {
        /* Enable PWM Module */
        pwmout_enable(obj);
    }
}

/** Get the duty cycle of PWM Waveform
 *
 * @param[in] obj  The PWM object
 * @return    Current duty cycle
 */
float pwmout_read(pwmout_t* obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);

    return obj->duty_cycle;
}

/** Set the period of PWM Waveform
 *
 * @param[in] obj      The PWM object
 * @param[in] seconds  New period in seconds
 * @return           void
 */
void pwmout_period(pwmout_t* obj, float seconds)
{
    pwmout_period_us(obj, seconds * 1000000.0f);
}

/** Set the period of PWM Waveform
 *
 * @param[in] obj    The PWM object
 * @param[in] value  New period in milliseconds
 * @return           void
 */
void pwmout_period_ms(pwmout_t* obj, int ms)
{
    pwmout_period_us(obj, ms * 1000);
}

/** Set the period of PWM Waveform
 *
 * @param[in] obj  The PWM object
 * @param[in] us   New period in microseconds
 * @return    void
 */
void pwmout_period_us(pwmout_t* obj, int us)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);

    /* Disable PWM Module */
    pwmout_disable(obj);

    /* TODO: Find and set the period */
    pwmout_set_period(obj, us);

    /* Update the changes */
    if (pwmout_init_hw(obj)) {
        /* Enable PWM Module */
        pwmout_enable(obj);
    }
}

/** Set the pulse width of PWM Waveform
 *
 * @param[in] obj      The PWM object
 * @param[in] seconds  New pulse width in seconds
 * @return    void
 */
void pwmout_pulsewidth(pwmout_t* obj, float seconds)
{
    pwmout_pulsewidth_us(obj, seconds * 1000000.0f);
}

/** Set the pulse width of PWM Waveform
 *
 * @param[in] obj  The PWM object
 * @param[in] ms   New pulse width in milliseconds
 * @return    void
 */
void pwmout_pulsewidth_ms(pwmout_t* obj, int ms)
{
    pwmout_pulsewidth_us(obj, ms * 1000);
}

/** Set the pulse width of PWM Waveform
 *
 * @param[in] obj  The PWM object
 * @param[in] us   New pulse width in microseconds
 * @return    void
 */
void pwmout_pulsewidth_us(pwmout_t* obj, int us)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);

    uint32_t us_pulse = us;

    /* Find the new duty cycle */
    double duty_cycle = us_pulse / ((double)obj->period * obj->us_per_cycle);

    /* This call updates pulse width as well as period */
    pwmout_write(obj, duty_cycle);
}

const PinMap *pwmout_pinmap()
{
    return PinMap_PWM;
}
