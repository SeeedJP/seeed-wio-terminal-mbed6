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
#include <cmsis.h>
#include "port_api.h"
#include "pinmap.h"
#include "gpio_api.h"
#include "port_drv.h"

uint32_t start_pin(PortName port)
{
    if(port < PortMax) { /* PortC value is 2*/
        return port * 32;
    } else {
        return (uint32_t)NC;
    }
}
void port_init(port_t *obj, PortName port, int mask, PinDirection dir)
{
    MBED_ASSERT(obj);
    struct port_config pin_conf;
    int i, j;
    int start;

    port_get_config_defaults(&pin_conf);
    switch (dir) {
        case PIN_INPUT :
            pin_conf.direction  = PORT_PIN_DIR_INPUT;
            break;
        case PIN_OUTPUT:
            pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
            break;
        case PIN_INPUT_OUTPUT:
            pin_conf.direction  = PORT_PIN_DIR_OUTPUT_WTH_READBACK;
            break;
        default:
            return;
    }

    port_group_registers_t *const port_base = (port_group_registers_t*)port_get_group_from_gpio_pin(port * 32);  // 32 pins in port // function reused to get the port base
    if(port_base == NULL) return; /* returns NULL if invalid*/
    obj->port_base = port_base;
    switch (port) {
        case PortA:
            obj->mask = (uint32_t)mask;
            break;
        case PortB:
            obj->mask = (uint32_t)mask;
            break;
        case PortC:
            obj->mask = (uint32_t)mask;
            break;
        case PortD:
            obj->mask = (uint32_t)mask;
            break;
        default:
            return;
    }
    start = start_pin(port);
    if(start == NC)
        return;
    obj->port = port;
    obj->direction = dir;
    obj->powersave = pin_conf.powersave;
    obj->mode = PORT_PIN_PULL_UP;

    for (i = start, j = 0; j < 32; i++, j++) {
        if (obj->mask & (1<<j)) {
            port_pin_set_config((PinName)i, &pin_conf);
        }
    }
    //obj->OUTCLR = &port_base->PORT_OUTCLR;
    //obj->OUTSET = &port_base->PORT_OUTSET;
    //obj->IN = &port_base->PORT_IN;
    //obj->OUT = &port_base->PORT_OUT;
}

void port_mode(port_t *obj, PinMode mode)
{
    MBED_ASSERT(obj);
    int i, j;
    int start;
    start = start_pin(obj->port);
    if(start == NC)
        return;
    for (i = start, j = 0; j < 32; i++, j++) {
        if (obj->mask & (1<<j)) {
            pin_mode((PinName)i , mode);
        }
    }
}

void port_dir(port_t *obj, PinDirection dir)
{
    MBED_ASSERT(obj);
    struct port_config pin_conf;
    int i, j;
    int start;

    switch (dir) {
        case PIN_INPUT :
            pin_conf.direction  = PORT_PIN_DIR_INPUT;
            break;
        case PIN_OUTPUT:
            pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
            break;
        case PIN_INPUT_OUTPUT:
            pin_conf.direction  = PORT_PIN_DIR_OUTPUT_WTH_READBACK;
            break;
        default:
            return;
    }
    start = start_pin(obj->port);
    if(start == NC)
        return;
    obj->direction = dir;
    pin_conf.input_pull = (enum port_pin_pull)obj->mode;
    pin_conf.powersave  = obj->powersave;

    for (i = start, j = 0; j < 32; i++, j++) {
        if (obj->mask & (1<<j)) {
            port_pin_set_config((PinName)i, &pin_conf);
        }
    }
}

void port_write(port_t *obj, int value)
{
    MBED_ASSERT(obj);
    int i;
    int start;
    start = start_pin(obj->port);
    if(start == NC)
        return;
    for (i = 0; i < 32 ; i++) {
        if (obj->mask & (1<<i)) {
            if (value & (1<<i)) {
                obj->port_base->PORT_OUTSET = 1 << i;
            } else {
                obj->port_base->PORT_OUTCLR = 1 << i;
            }
        }
    }
}

int port_read(port_t *obj)
{
    MBED_ASSERT(obj);
    if (obj->direction == PIN_OUTPUT) {
        return (obj->port_base->PORT_OUT & obj->mask);
    } else {
        return (obj->port_base->PORT_IN & obj->mask);
    }
}
