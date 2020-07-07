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
#ifndef MBED_OBJECTS_H
#define MBED_OBJECTS_H

#include "cmsis.h"
#include "PortNames.h"
#include "PeripheralNames.h"
#include "gpio_object.h"
#include "extint.h"
#include "us_ticker_defines.h"
#include "i2c_master.h"
#include "i2c_slave.h"
#include "adc_feature.h"

#ifdef __cplusplus
extern "C" {
#endif

struct waveconfig_t {
    /** Internal clock signals selection. */
    uint32_t ul_intclock;
    /** Waveform frequency (in Hz). */
    uint16_t us_frequency;
    /** Duty cycle in percent (positive).*/
    uint16_t us_dutycycle;
};

struct gpio_irq_s {
    uint8_t irqmask;
    uint32_t port;
    uint32_t pin;
    uint32_t ch;
    struct extint_chan_conf config_extint_chan;
};

struct port_s {
    PortName port;
    port_group_registers_t *port_base;
    uint32_t mask;
    uint8_t powersave;
    uint8_t mode;
    uint8_t direction;
};

#if DEVICE_SERIAL
struct serial_s {
    sercom_registers_t *usart;
    uint32_t index;
    uint32_t parity;
    uint32_t stopbits;
    uint32_t character_size;
    uint32_t mux_setting;
    uint32_t baudrate;
    PinName pins[4];
#if DEVICE_SERIAL_ASYNCH
    uint32_t events;
#endif
};
#endif

#if DEVICE_ANALOGIN
struct analogin_s {
    struct adc_config config_adc;
    uint8_t adc_id;
};
#endif

#if DEVICE_PWMOUT
struct pwmout_s {
    uint32_t channel;
    uint32_t ioline;
    uint32_t pin;
    uint32_t prescalarindex;
    struct waveconfig_t waveconfig;
};
#endif

#if DEVICE_I2C
struct i2c_s {
    struct i2c_master_module master;
    struct i2c_slave_module slave;
    uint8_t mode;
    uint32_t baud_rate;
    uint32_t baud_rate_high_speed;
    uint8_t start_pending;
    PinName pins[2];
#if DEVICE_I2C_ASYNCH
    uint32_t events;
    uint32_t handler;
    struct i2c_master_packet wr_packet;
    struct i2c_master_packet rd_packet;
#endif
};
#endif

#if DEVICE_SPI
struct spi_s {
    sercom_registers_t *spi;
    uint8_t mode;
    PinName pins[4];
#if DEVICE_SPI_ASYNCH
    uint8_t status;
    uint32_t mask;
    uint32_t event;
    void *tx_buffer;
    void *rx_buffer;
    uint8_t dma_usage;
#endif
};
#endif

#ifdef __cplusplus
}
#endif

#endif
