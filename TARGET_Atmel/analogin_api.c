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
#include "analogin_api.h"

#include "cmsis.h"
#include "pinmap.h"
#include "PeripheralPins.h"
#include "adc_drv.h"
#include "status_codes.h"

extern uint8_t g_sys_init;
struct adc_module adc_instance[2];

void analogin_init(analogin_t *obj, PinName pin)
{
    MBED_ASSERT(obj);
    if (g_sys_init == 0) {
        system_init();
        g_sys_init = 1;
    }
    uint32_t pos_input;
    static bool init_flag[2] = {false, false};
    int function;

    uint32_t peripheral = pinmap_find_peripheral(pin, PinMap_ADC);
    adc_registers_t *adc_module = (adc_registers_t *)(peripheral & ~0xf);
    obj->adc_id = (uint8_t)(peripheral & 0xf);

    MBED_ASSERT(adc_module != (adc_registers_t *)NC);
    function = pinmap_find_function(pin, PinMap_ADC);

    adc_get_config_defaults(&(obj->config_adc));
    obj->config_adc.reference = ADC_REFERENCE_INTVCC1;
    obj->config_adc.positive_input = (enum adc_positive_input)((function >> 4) & 0xf);
    obj->config_adc.negative_input = ADC_NEGATIVE_INPUT_GND;

    if (init_flag[obj->adc_id] == false) {  // ADC init and enable to be done only once.
        adc_init(&adc_instance[obj->adc_id], adc_module, &(obj->config_adc));
        adc_enable(&adc_instance[obj->adc_id]);
        init_flag[obj->adc_id] = true;
    }

    struct system_pinmux_config config;
    system_pinmux_get_config_defaults(&config);

    /* Analog functions are all on MUX setting B */
    config.input_pull   = SYSTEM_PINMUX_PIN_PULL_NONE;
    config.mux_position = 1;

    system_pinmux_pin_set_config(pin, &config);
}

uint16_t analogin_read_u16(analogin_t *obj)
{
    MBED_ASSERT(obj);
    uint16_t result;
    adc_set_positive_input(&adc_instance[obj->adc_id], obj->config_adc.positive_input);
    adc_set_negative_input(&adc_instance[obj->adc_id], obj->config_adc.negative_input);
    adc_start_conversion(&adc_instance[obj->adc_id]);
    do {
    } while(adc_read(&adc_instance[obj->adc_id], &result) == STATUS_BUSY);   // 12 bit value

    return (uint16_t)(((uint32_t)result * 0xFFFF) / 0x0FFF);  // for normalizing to 16 bit value
}

float analogin_read(analogin_t *obj)
{
    MBED_ASSERT(obj);
    uint16_t value = analogin_read_u16(obj);
    return (float)value * (1.0f / (float)0xFFFF);
}

const PinMap *analogin_pinmap()
{
    return PinMap_ADC;
}
