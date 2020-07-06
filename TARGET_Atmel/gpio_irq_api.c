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
#include "cmsis.h"

#include "gpio_irq_api.h"
#include "gpio_api.h"
#include "mbed_error.h"
#include "extint.h"
#include "pinmap.h"
#include "PeripheralPins.h"
#include "port_drv.h"
#include "system_interrupt_features.h"

#define IRQ_RISE_POSITION 1
#define IRQ_FALL_POSITION 2
#define CHANNEL_NUM 16
#define pEXT_CONF(obj) (obj->config_extint_chan)
static uint32_t channel_ids[CHANNEL_NUM] = {0};
static gpio_irq_handler irq_handler[CHANNEL_NUM];
uint8_t ext_int_pins[EIC_NUMBER_OF_INTERRUPTS] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void gpio_irq(uint32_t ch)
{
    uint32_t mask;
    gpio_irq_event event;
    port_group_registers_t *port_base;

    port_base = (port_group_registers_t*)port_get_group_from_gpio_pin(ext_int_pins[ch]);
    mask = gpio_set((PinName)ext_int_pins[ch]);
    if ((port_base->PORT_IN & mask) != 0) {
        event = IRQ_RISE;
    } else {
        event = IRQ_FALL;
    }
    if (irq_handler[ch]) {
        irq_handler[ch](channel_ids[ch], event);
    }
}

int gpio_irq_init(gpio_irq_t *obj, PinName pin, gpio_irq_handler handler, uint32_t id)
{
    MBED_ASSERT(obj);
    if (pin == NC)
        return -1;

    IRQn_Type irq_n = (IRQn_Type)0;
    uint32_t vector = 0;
    int int_channel = 0;

    obj->pin = pin;

    extint_chan_get_config_defaults(&pEXT_CONF(obj));
    pEXT_CONF(obj).gpio_pin           = (uint32_t)pin;
    pEXT_CONF(obj).gpio_pin_mux       = 0;   // mux setting for ext int is 0
    pEXT_CONF(obj).gpio_pin_pull      = EXTINT_PULL_UP;
    pEXT_CONF(obj).detection_criteria = EXTINT_DETECT_NONE;

    int_channel = pinmap_find_peripheral(pin, PinMap_EXTINT);
    if (int_channel == NC) {
        return -1;
    }
    extint_chan_set_config(int_channel, &pEXT_CONF(obj));
    ext_int_pins[int_channel] = pin;
    irq_handler[int_channel] = handler;

    irq_n = SYSTEM_INTERRUPT_MODULE_EIC_EXTINT_0 + int_channel;
    vector = (uint32_t)gpio_irq;
    NVIC_DisableIRQ(irq_n);
    NVIC_ClearPendingIRQ(irq_n);
    extint_register_callback(vector, int_channel, EXTINT_CALLBACK_TYPE_DETECT);
    NVIC_EnableIRQ(irq_n);
    obj->ch = int_channel;
    channel_ids[int_channel] = id;
    obj->irqmask = 0;

    return 0;
}

void gpio_irq_free(gpio_irq_t *obj)
{
    MBED_ASSERT(obj);
    eic_registers_t *const eic = _extint_get_eic_from_channel(obj->ch);
    channel_ids[obj->ch] = 0;
    irq_handler[obj->ch] = NULL;
    eic->EIC_INTENCLR = (1UL << obj->ch);
}

void gpio_irq_set(gpio_irq_t *obj, gpio_irq_event event, uint32_t enable)
{
    MBED_ASSERT(obj);
    eic_registers_t *const eic = _extint_get_eic_from_channel(obj->ch);
    if (enable) {
        if (event == IRQ_RISE) {
            obj->irqmask |= IRQ_RISE_POSITION;
        } else if (event == IRQ_FALL) {
            obj->irqmask |= IRQ_FALL_POSITION;
        }
        eic->EIC_INTENSET = (1UL << obj->ch);
    } else {
        if (event == IRQ_RISE) {
            obj->irqmask &= ~IRQ_RISE_POSITION;
        } else if (event == IRQ_FALL) {
            obj->irqmask &= ~IRQ_FALL_POSITION;
        }
    }

    if (obj->irqmask == (IRQ_RISE_POSITION | IRQ_FALL_POSITION)) {
        pEXT_CONF(obj).detection_criteria = EXTINT_DETECT_BOTH;
    } else if (obj->irqmask == IRQ_RISE_POSITION) {
        pEXT_CONF(obj).detection_criteria = EXTINT_DETECT_RISING;
    } else if (obj->irqmask == IRQ_FALL_POSITION) {
        pEXT_CONF(obj).detection_criteria = EXTINT_DETECT_FALLING;
    } else {
        pEXT_CONF(obj).detection_criteria = EXTINT_DETECT_NONE;
        eic->EIC_INTENCLR = (1UL << obj->ch);
    }

    extint_chan_set_config(obj->ch, &pEXT_CONF(obj));
}

void gpio_irq_enable(gpio_irq_t *obj)
{
    MBED_ASSERT(obj);
    eic_registers_t *const eic = _extint_get_eic_from_channel(obj->ch);
    NVIC_EnableIRQ(SYSTEM_INTERRUPT_MODULE_EIC_EXTINT_0 + obj->ch);
    eic->EIC_INTENSET = (1UL << obj->ch);
}

void gpio_irq_disable(gpio_irq_t *obj)
{
    MBED_ASSERT(obj);
    eic_registers_t *const eic = _extint_get_eic_from_channel(obj->ch);
    eic->EIC_INTENCLR = (1UL << obj->ch);
    NVIC_DisableIRQ(SYSTEM_INTERRUPT_MODULE_EIC_EXTINT_0 + obj->ch);
}
