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
#include <string.h>
#include "mbed_assert.h"
#include "cmsis.h"
#include "serial_api.h"
#include "pinmap.h"
#include "PeripheralPins.h"
#include "sercom_drv.h"
#include "usart.h"
#include "pinmap_function.h"

#define USART_TX_INDEX		0
#define USART_RX_INDEX		1
#define USART_RXFLOW_INDEX	2
#define USART_TXFLOW_INDEX	3

#if DEVICE_SERIAL_ASYNCH
#define pUSART_S(obj)			obj->serial.usart
#define pSERIAL_S(obj)			((struct serial_s*)&(obj->serial))
#else
#define pUSART_S(obj)			obj->usart
#define pSERIAL_S(obj)			((struct serial_s*)obj)
#endif
#define _USART(obj)			pUSART_S(obj)->USART_INT
#define USART_NUM 6
#define SUPPRESS_WARNING(a) (void)a

uint8_t serial_get_index(serial_t *obj);
IRQn_Type get_serial_irq_num (serial_t *obj);

static uint32_t serial_irq_ids[USART_NUM] = {0};
static uart_irq_handler irq_handler;

int stdio_uart_inited = 0;
serial_t stdio_uart;

extern uint8_t g_sys_init;

static inline void usart_syncing(serial_t *obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);
    while(_USART(obj).SERCOM_SYNCBUSY);
}

static inline void enable_usart(serial_t *obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);

    /* Wait until synchronization is complete */
    usart_syncing(obj);

    /* Enable USART module */
    _USART(obj).SERCOM_CTRLA |= SERCOM_USART_INT_CTRLA_ENABLE(1);
}

static inline void disable_usart(serial_t *obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);

    /* Wait until synchronization is complete */
    usart_syncing(obj);

    /* Disable USART module */
    _USART(obj).SERCOM_CTRLA &= ~SERCOM_USART_INT_CTRLA_ENABLE(1);
}

static inline void reset_usart(serial_t *obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);

    disable_usart(obj);

    /* Wait until synchronization is complete */
    usart_syncing(obj);

    /* Reset module */
    _USART(obj).SERCOM_CTRLA = SERCOM_USART_INT_CTRLA_SWRST(1);
    SUPPRESS_WARNING(reset_usart);
}

uint32_t serial_find_mux_settings (serial_t *obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);
    uint32_t mux_setting = 0;
    uint32_t pinpad[4] = {0};
    uint8_t i = 0;
    uint32_t sercom_index = pinmap_merge_sercom(pSERIAL_S(obj)->pins[0], pSERIAL_S(obj)->pins[1]);

    for (i = 0; i < 4 ; i++) {
        pinpad[i] = pinmap_pad_sercom(pSERIAL_S(obj)->pins[i], sercom_index);
    }

    switch(pinpad[USART_RX_INDEX]) {
        case 0:
            mux_setting |= SERCOM_USART_INT_CTRLA_RXPO(0);
            break;
        case 1:
            mux_setting |= SERCOM_USART_INT_CTRLA_RXPO(1);
            break;
        case 2:
            mux_setting |= SERCOM_USART_INT_CTRLA_RXPO(2);
            break;
        case 3:
            mux_setting |= SERCOM_USART_INT_CTRLA_RXPO(3);
            break;
    }

    if ((pSERIAL_S(obj)->pins[USART_RXFLOW_INDEX] == NC) && (pSERIAL_S(obj)->pins[USART_TXFLOW_INDEX] == NC)) {
        if (pinpad[USART_TX_INDEX] == 0) {
            mux_setting |= SERCOM_USART_INT_CTRLA_TXPO(0);
        } else if(pinpad[USART_TX_INDEX] == 2) {
            mux_setting |= SERCOM_USART_INT_CTRLA_TXPO(1);
        } else {
        }
    } else { // for hardware flow control and uart // expecting the tx in pad 0, rts in pad2 and cts in pad 3
        if((pinpad[USART_TX_INDEX] == 0) && (pinpad[USART_RXFLOW_INDEX]/*rts pin*/ == 2) && (pinpad[USART_TXFLOW_INDEX] /*cts pin*/ == 3)) {
            mux_setting |= SERCOM_USART_INT_CTRLA_TXPO(2);
        }
    }
    return mux_setting;
}

static enum status_code usart_set_config_default(serial_t *obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);
    /* Index for generic clock */
    uint32_t sercom_index = _sercom_get_sercom_inst_index(pUSART_S(obj));
    uint32_t chan_index[] = {
        GCLK_PCHCTRL_ID_7_GCLK_SERCOM0_CORE,
        GCLK_PCHCTRL_ID_8_GCLK_SERCOM1_CORE,
        GCLK_PCHCTRL_ID_23_GCLK_SERCOM2_CORE,
        GCLK_PCHCTRL_ID_24_GCLK_SERCOM3_CORE,
        GCLK_PCHCTRL_ID_34_GCLK_SERCOM4_CORE,
        GCLK_PCHCTRL_ID_35_GCLK_SERCOM5_CORE,
        GCLK_PCHCTRL_ID_36_GCLK_SERCOM6_CORE,
        GCLK_PCHCTRL_ID_37_GCLK_SERCOM7_CORE,
    };

    /* Cache new register values to minimize the number of register writes */
    uint32_t ctrla = 0;
    uint32_t ctrlb = 0;
    uint16_t baud  = 0;

    enum sercom_asynchronous_operation_mode mode = SERCOM_ASYNC_OPERATION_MODE_ARITHMETIC;
    enum sercom_asynchronous_sample_num sample_num = SERCOM_ASYNC_SAMPLE_NUM_16;

    /* Set data order, internal muxing, and clock polarity */
    ctrla = (uint32_t)USART_DATAORDER_LSB |         // data order
            (uint32_t)pSERIAL_S(obj)->mux_setting;  // mux setting  // clock polarity is not used


    /* Get baud value from mode and clock */
    _sercom_get_async_baud_val(pSERIAL_S(obj)->baudrate,
            system_gclk_chan_get_hz(chan_index[sercom_index]), &baud, mode, sample_num);  // for asynchronous transfer mode

    /* Wait until synchronization is complete */
    usart_syncing(obj);

    /*Set baud val */
    _USART(obj).SERCOM_BAUD = baud;

    /* Set sample mode */
    ctrla |= USART_TRANSFER_ASYNCHRONOUSLY;

    /* for disabled external clock source */
    ctrla |= SERCOM_USART_INT_CTRLA_MODE(0x1);

    /* Set stopbits, character size and enable transceivers */
    ctrlb = (uint32_t)pSERIAL_S(obj)->stopbits | (uint32_t)pSERIAL_S(obj)->character_size |
            SERCOM_USART_INT_CTRLB_RXEN(1) | SERCOM_USART_INT_CTRLB_TXEN(1);  /*transmitter and receiver enable*/
    if (pSERIAL_S(obj)->pins[USART_RX_INDEX] == NC) { /* if pin is NC, have to disable the corresponding transmitter/receiver part */
        ctrlb &= ~SERCOM_USART_INT_CTRLB_RXEN(1);  /* receiver disable */
    }
    if (pSERIAL_S(obj)->pins[USART_TX_INDEX] == NC) {
        ctrlb &= ~SERCOM_USART_INT_CTRLB_TXEN(1);  /* transmitter disable  */
    }

    /* Check parity mode bits */
    if (pSERIAL_S(obj)->parity != USART_PARITY_NONE) {
        ctrla |= SERCOM_USART_INT_CTRLA_FORM(1);
        ctrlb |= pSERIAL_S(obj)->parity;
    } else {
        ctrla |= SERCOM_USART_INT_CTRLA_FORM(0);
    }

    /* Wait until synchronization is complete */
    usart_syncing(obj);

    /* Write configuration to CTRLB */
    _USART(obj).SERCOM_CTRLB = ctrlb;

    /* Wait until synchronization is complete */
    usart_syncing(obj);

    /* Write configuration to CTRLA */
    _USART(obj).SERCOM_CTRLA = ctrla;

    return STATUS_OK;
}

void get_default_serial_values(serial_t *obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);
    /* Set default config to object */
    pSERIAL_S(obj)->parity = USART_PARITY_NONE;
    pSERIAL_S(obj)->stopbits = USART_STOPBITS_1;
    pSERIAL_S(obj)->character_size = USART_CHARACTER_SIZE_8BIT;
    pSERIAL_S(obj)->baudrate = 9600;
    pSERIAL_S(obj)->mux_setting = USART_RX_1_TX_2_XCK_3;
};

void serial_init(serial_t *obj, PinName tx, PinName rx)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);
    if (g_sys_init == 0) {
        system_init();
        g_sys_init = 1;
    }
    struct system_gclk_chan_config gclk_chan_conf;
    uint32_t uart_base;
    struct sercom_apb_clock {
        enum system_clock_apb_bus apb_bus;
        uint32_t mask;
    } apb_index[] = {
        { SYSTEM_CLOCK_APB_APBA, MCLK_APBAMASK_SERCOM0(1), },
        { SYSTEM_CLOCK_APB_APBA, MCLK_APBAMASK_SERCOM1(1), },
        { SYSTEM_CLOCK_APB_APBB, MCLK_APBBMASK_SERCOM2(1), },
        { SYSTEM_CLOCK_APB_APBB, MCLK_APBBMASK_SERCOM3(1), },
        { SYSTEM_CLOCK_APB_APBD, MCLK_APBDMASK_SERCOM4(1), },
        { SYSTEM_CLOCK_APB_APBD, MCLK_APBDMASK_SERCOM5(1), },
        { SYSTEM_CLOCK_APB_APBD, MCLK_APBDMASK_SERCOM6(1), },
        { SYSTEM_CLOCK_APB_APBD, MCLK_APBDMASK_SERCOM7(1), },
    };
    uint32_t chan_index[] = {
        GCLK_PCHCTRL_ID_7_GCLK_SERCOM0_CORE,
        GCLK_PCHCTRL_ID_8_GCLK_SERCOM1_CORE,
        GCLK_PCHCTRL_ID_23_GCLK_SERCOM2_CORE,
        GCLK_PCHCTRL_ID_24_GCLK_SERCOM3_CORE,
        GCLK_PCHCTRL_ID_34_GCLK_SERCOM4_CORE,
        GCLK_PCHCTRL_ID_35_GCLK_SERCOM5_CORE,
        GCLK_PCHCTRL_ID_36_GCLK_SERCOM6_CORE,
        GCLK_PCHCTRL_ID_37_GCLK_SERCOM7_CORE,
    };
    uint32_t sercom_index = 0;
    uint32_t muxsetting = 0;

    get_default_serial_values(obj);

    pSERIAL_S(obj)->pins[USART_TX_INDEX] = tx;
    pSERIAL_S(obj)->pins[USART_RX_INDEX] = rx;
    pSERIAL_S(obj)->pins[USART_RXFLOW_INDEX] = NC;
    pSERIAL_S(obj)->pins[USART_TXFLOW_INDEX] = NC;

    muxsetting = serial_find_mux_settings(obj);  // getting mux setting from pins
    sercom_index = pinmap_merge_sercom(tx, rx);  // same variable sercom_index reused for optimization
    if (sercom_index == (uint32_t)NC) {
        /*expecting a valid value for sercom index*/
        return;
    }
    sercom_index &= 0x0F;
    uart_base = pinmap_peripheral_sercom(NC, sercom_index);
    pUSART_S(obj) = (sercom_registers_t *)uart_base;

    /* Disable USART module */
    disable_usart(obj);

    if (_USART(obj).SERCOM_CTRLA & SERCOM_USART_INT_CTRLA_SWRST(1)) {
        return;  /* The module is busy resetting itself */
    }

    if (_USART(obj).SERCOM_CTRLA & SERCOM_USART_INT_CTRLA_ENABLE(1)) {
        return;    /* Check the module is enabled */
    }

    /* Turn on module in PM */
    system_apb_clock_set_mask(apb_index[sercom_index].apb_bus, apb_index[sercom_index].mask);

    /* Set up the GCLK for the module */
    gclk_chan_conf.source_generator = GCLK_GENERATOR_1;
    system_gclk_chan_set_config(chan_index[sercom_index], &gclk_chan_conf);
    system_gclk_chan_enable(chan_index[sercom_index]);
    sercom_set_gclk_generator(GCLK_GENERATOR_1, false);

    pSERIAL_S(obj)->mux_setting = muxsetting;
    /* Set configuration according to the config struct */
    usart_set_config_default(obj);

    struct system_pinmux_config pin_conf;
    pin_conf.direction = SYSTEM_PINMUX_PIN_DIR_INPUT;
    pin_conf.input_pull = SYSTEM_PINMUX_PIN_PULL_NONE;
    pin_conf.powersave    = false;

    /* Configure the SERCOM pins according to the user configuration */
    for (uint8_t pad = 0; pad < 4; pad++) {
        uint32_t current_pin = pSERIAL_S(obj)->pins[pad];
        if (current_pin != (uint32_t)NC) {
            pin_conf.mux_position = pinmap_function_sercom((PinName)current_pin, sercom_index);
            if ((uint8_t)NC != pin_conf.mux_position) {
                system_pinmux_pin_set_config(current_pin, &pin_conf);
            }
        }
    }

    if (sercom_index == STDIO_UART) {
        stdio_uart_inited = 1;
        memcpy(&stdio_uart, obj, sizeof(serial_t));
    }
    /* Wait until synchronization is complete */
    usart_syncing(obj);

    /* Enable USART module */
    enable_usart(obj);
}

void serial_free(serial_t *obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);
    struct system_pinmux_config pin_conf;
    serial_irq_ids[serial_get_index(obj)] = 0;
    disable_usart(obj);

    pin_conf.direction = SYSTEM_PINMUX_PIN_DIR_INPUT;
    pin_conf.input_pull = SYSTEM_PINMUX_PIN_PULL_UP;
    pin_conf.powersave    = false;
    pin_conf.mux_position = SYSTEM_PINMUX_GPIO;
    /* Configure the SERCOM pins according to the user configuration */
    for (uint8_t pad = 0; pad < 4; pad++) {
        uint32_t current_pin = pSERIAL_S(obj)->pins[pad];
        if (current_pin != (uint32_t)NC) {
            system_pinmux_pin_set_config(current_pin, &pin_conf);
        }
    }
}

void serial_baud(serial_t *obj, int baudrate)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);
    MBED_ASSERT((baudrate == 110) || (baudrate == 150) || (baudrate == 300) || (baudrate == 1200) ||
                (baudrate == 2400) || (baudrate == 4800) || (baudrate == 9600) || (baudrate == 19200) || (baudrate == 38400) ||
                (baudrate == 57600) || (baudrate == 115200) || (baudrate == 230400) || (baudrate == 460800) || (baudrate == 921600) );

    struct system_gclk_chan_config gclk_chan_conf;
    uint32_t chan_index[] = {
        GCLK_PCHCTRL_ID_7_GCLK_SERCOM0_CORE,
        GCLK_PCHCTRL_ID_8_GCLK_SERCOM1_CORE,
        GCLK_PCHCTRL_ID_23_GCLK_SERCOM2_CORE,
        GCLK_PCHCTRL_ID_24_GCLK_SERCOM3_CORE,
        GCLK_PCHCTRL_ID_34_GCLK_SERCOM4_CORE,
        GCLK_PCHCTRL_ID_35_GCLK_SERCOM5_CORE,
        GCLK_PCHCTRL_ID_36_GCLK_SERCOM6_CORE,
        GCLK_PCHCTRL_ID_37_GCLK_SERCOM7_CORE,
    };
    uint16_t baud  = 0;
    uint32_t sercom_index = 0;
    enum sercom_asynchronous_operation_mode mode = SERCOM_ASYNC_OPERATION_MODE_ARITHMETIC;
    enum sercom_asynchronous_sample_num sample_num = SERCOM_ASYNC_SAMPLE_NUM_16;

    pSERIAL_S(obj)->baudrate = baudrate;
    disable_usart(obj);

    sercom_index = _sercom_get_sercom_inst_index(pUSART_S(obj));
    gclk_chan_conf.source_generator = GCLK_GENERATOR_1;
    system_gclk_chan_set_config(chan_index[sercom_index], &gclk_chan_conf);
    system_gclk_chan_enable(chan_index[sercom_index]);
    sercom_set_gclk_generator(GCLK_GENERATOR_1, false);

    /* Get baud value from mode and clock */
    _sercom_get_async_baud_val(pSERIAL_S(obj)->baudrate,
            system_gclk_chan_get_hz(chan_index[sercom_index]), &baud, mode, sample_num);

    /* Wait until synchronization is complete */
    usart_syncing(obj);

    /*Set baud val */
    _USART(obj).SERCOM_BAUD = baud;
    /* Wait until synchronization is complete */
    usart_syncing(obj);

    enable_usart(obj);
}

void serial_format(serial_t *obj, int data_bits, SerialParity parity, int stop_bits)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);
    MBED_ASSERT((stop_bits == 1) || (stop_bits == 2));
    MBED_ASSERT((parity == ParityNone) || (parity == ParityOdd) || (parity == ParityEven));
    MBED_ASSERT((data_bits == 5) || (data_bits == 6) || (data_bits == 7) || (data_bits == 8) /*|| (data_bits == 9)*/);

    /* Cache new register values to minimize the number of register writes */
    uint32_t ctrla = 0;
    uint32_t ctrlb = 0;

    disable_usart(obj);

    ctrla = _USART(obj).SERCOM_CTRLA;
    ctrlb = _USART(obj).SERCOM_CTRLB;

    ctrla &= ~(SERCOM_USART_INT_CTRLA_FORM_Msk);
    ctrlb &= ~(SERCOM_USART_INT_CTRLB_CHSIZE_Msk);
    ctrlb &= ~(SERCOM_USART_INT_CTRLB_SBMODE_Msk);
    ctrlb &= ~(SERCOM_USART_INT_CTRLB_PMODE_Msk);

    switch (stop_bits) {
        case 1:
            pSERIAL_S(obj)->stopbits = USART_STOPBITS_1;
            break;
        case 2:
            pSERIAL_S(obj)->stopbits = USART_STOPBITS_2;
            break;
        default:
            pSERIAL_S(obj)->stopbits = USART_STOPBITS_1;
    }

    switch (parity) {
        case ParityNone:
            pSERIAL_S(obj)->parity = USART_PARITY_NONE;
            break;
        case ParityOdd:
            pSERIAL_S(obj)->parity = USART_PARITY_ODD;
            break;
        case ParityEven:
            pSERIAL_S(obj)->parity = USART_PARITY_EVEN;
            break;
        default:
            pSERIAL_S(obj)->parity = USART_PARITY_NONE;
    }

    switch (data_bits) {
        case 5:
            pSERIAL_S(obj)->character_size = USART_CHARACTER_SIZE_5BIT;
            break;
        case 6:
            pSERIAL_S(obj)->character_size = USART_CHARACTER_SIZE_6BIT;
            break;
        case 7:
            pSERIAL_S(obj)->character_size = USART_CHARACTER_SIZE_7BIT;
            break;
        case 8:
            pSERIAL_S(obj)->character_size = USART_CHARACTER_SIZE_8BIT;
            break;  //  9 bit transfer not required in mbed
        default:
            pSERIAL_S(obj)->character_size = USART_CHARACTER_SIZE_8BIT;
    }


    /* Set stopbits, character size and enable transceivers */
    ctrlb |= (pSERIAL_S(obj)->stopbits | pSERIAL_S(obj)->character_size);

    /* Check parity mode bits */
    if (pSERIAL_S(obj)->parity != USART_PARITY_NONE) {
        ctrla |= SERCOM_USART_INT_CTRLA_FORM(1);
        ctrlb |= pSERIAL_S(obj)->parity;
    } else {
        ctrla |= SERCOM_USART_INT_CTRLA_FORM(0);
    }

    /* Write configuration to CTRLB */
    _USART(obj).SERCOM_CTRLB = ctrlb;

    /* Wait until synchronization is complete */
    usart_syncing(obj);

    /* Write configuration to CTRLA */
    _USART(obj).SERCOM_CTRLA = ctrla;

    /* Wait until synchronization is complete */
    usart_syncing(obj);

    enable_usart(obj);
}

#if DEVICE_SERIAL_FC

void serial_set_flow_control(serial_t *obj, FlowControl type, PinName rxflow, PinName txflow)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);
    uint32_t muxsetting = 0;
    uint32_t sercom_index = 0;

    pSERIAL_S(obj)->pins[USART_RXFLOW_INDEX] = rxflow;
    pSERIAL_S(obj)->pins[USART_TXFLOW_INDEX] = txflow;
    muxsetting = serial_find_mux_settings(obj);  // getting mux setting from pins
    sercom_index = pinmap_merge_sercom(pSERIAL_S(obj)->pins[USART_TX_INDEX], pSERIAL_S(obj)->pins[USART_RX_INDEX]);  // same variable sercom_index reused for optimization
    if (sercom_index == (uint32_t)NC) {
        /*expecting a valid value for sercom index*/
        return;
    }

    disable_usart(obj);

    /* Set configuration according to the config struct */
    pSERIAL_S(obj)->mux_setting = muxsetting;  // mux setting to be changed for configuring hardware control
    usart_set_config_default(obj);

    struct system_pinmux_config pin_conf;
    pin_conf.direction = SYSTEM_PINMUX_PIN_DIR_INPUT;
    pin_conf.input_pull = SYSTEM_PINMUX_PIN_PULL_NONE;
    pin_conf.powersave    = false;

    for (uint8_t pad = 0; pad < 2; pad++) {  // setting for rx and tx
        uint32_t current_pin = pSERIAL_S(obj)->pins[pad];
        if (current_pin != (uint32_t)NC) {
            pin_conf.mux_position = pinmap_function_sercom((PinName)current_pin, sercom_index);
            if ((uint8_t)NC != pin_conf.mux_position) {
                system_pinmux_pin_set_config(current_pin, &pin_conf);
            }
        }
    }
    if((FlowControlRTS == type) || (FlowControlRTSCTS== type))  {
        if (pSERIAL_S(obj)->pins[USART_RXFLOW_INDEX] != NC) {
            pin_conf.direction = SYSTEM_PINMUX_PIN_DIR_OUTPUT; // setting for rxflow
            pin_conf.input_pull = SYSTEM_PINMUX_PIN_PULL_UP;
            pin_conf.mux_position = pinmap_function_sercom(pSERIAL_S(obj)->pins[USART_RXFLOW_INDEX] , sercom_index);
            if ((uint8_t)NC != pin_conf.mux_position) {
                system_pinmux_pin_set_config(pSERIAL_S(obj)->pins[USART_RXFLOW_INDEX], &pin_conf);
            }
        }
    }
    if((FlowControlCTS == type) || (FlowControlRTSCTS== type)) {
        if (pSERIAL_S(obj)->pins[USART_TXFLOW_INDEX] != NC) {
            pin_conf.direction = SYSTEM_PINMUX_PIN_DIR_INPUT; // setting for txflow
            pin_conf.input_pull = SYSTEM_PINMUX_PIN_PULL_UP;
            pin_conf.mux_position = pinmap_function_sercom(pSERIAL_S(obj)->pins[USART_TXFLOW_INDEX] , sercom_index);
            if ((uint8_t)NC != pin_conf.mux_position) {
                system_pinmux_pin_set_config(pSERIAL_S(obj)->pins[USART_TXFLOW_INDEX], &pin_conf);
            }
        }
    }
    enable_usart(obj);
}

#endif  //DEVICE_SERIAL_FC

void serial_break_set(serial_t *obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);
    struct system_pinmux_config pin_conf;
    pin_conf.direction = SYSTEM_PINMUX_PIN_DIR_OUTPUT;
    pin_conf.input_pull = SYSTEM_PINMUX_PIN_PULL_NONE;
    pin_conf.mux_position = SYSTEM_PINMUX_GPIO;
    pin_conf.powersave    = false;

    if (pSERIAL_S(obj)->pins[USART_TX_INDEX] != NC) {
        system_pinmux_pin_set_config(pSERIAL_S(obj)->pins[USART_TX_INDEX], &pin_conf);
    }
}

void serial_break_clear(serial_t *obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);
    uint32_t sercom_index = pinmap_merge_sercom(pSERIAL_S(obj)->pins[USART_TX_INDEX], pSERIAL_S(obj)->pins[USART_RX_INDEX]);

    struct system_pinmux_config pin_conf;
    pin_conf.direction = SYSTEM_PINMUX_PIN_DIR_INPUT;
    pin_conf.input_pull = SYSTEM_PINMUX_PIN_PULL_NONE;
    pin_conf.powersave    = false;

    if (pSERIAL_S(obj)->pins[USART_TX_INDEX] != NC) {
        pin_conf.mux_position = pinmap_function_sercom(pSERIAL_S(obj)->pins[USART_TX_INDEX], sercom_index);
        if ((uint8_t)NC != pin_conf.mux_position) {
            system_pinmux_pin_set_config(pSERIAL_S(obj)->pins[USART_TX_INDEX], &pin_conf);
        }
    }
}

void serial_pinout_tx(PinName tx)
{
    pinmap_pinout(tx, PinMap_SERCOM);
}

/******************************************************************************
 * INTERRUPTS HANDLING
 ******************************************************************************/
inline uint8_t serial_get_index(serial_t *obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);
    switch ((int)pUSART_S(obj)) {
        case UART_0:
            return 0;
        case UART_1:
            return 1;
        case UART_2:
            return 2;
        case UART_3:
            return 3;
        case UART_4:
            return 4;
        case UART_5:
            return 5;
    }
    return 0;
}

static void uart_irq(const uint8_t instance, const uint8_t offset)
{
    serial_t *obj = _sercom_instances[instance];
    uint16_t interrupt_status;
    interrupt_status = _USART(obj).SERCOM_INTFLAG;
    interrupt_status &= _USART(obj).SERCOM_INTENSET;

    if (offset == 1 || interrupt_status & SERCOM_USART_INT_INTFLAG_TXC(1)) { // for transmit complete
        _USART(obj).SERCOM_INTFLAG = SERCOM_USART_INT_INTFLAG_TXC(1);
        if (irq_handler && serial_irq_ids[instance]) {
            irq_handler(serial_irq_ids[instance], TxIrq);
        }
    }
    if (offset == 2 || interrupt_status & SERCOM_USART_INT_INTFLAG_RXC(1)) { // for receive complete
        _USART(obj).SERCOM_INTFLAG = SERCOM_USART_INT_INTFLAG_RXC(1);
        if (irq_handler && serial_irq_ids[instance]) {
            irq_handler(serial_irq_ids[instance], RxIrq);
        }
    }
}

void serial_irq_handler(serial_t *obj, uart_irq_handler handler, uint32_t id)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);
    irq_handler = handler;
    serial_irq_ids[serial_get_index(obj)] = id;
}

IRQn_Type get_serial_irq_num (serial_t *obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);
    switch ((int)pUSART_S(obj)) {
        case UART_0:
            return SERCOM0_0_IRQn;
        case UART_1:
            return SERCOM1_0_IRQn;
        case UART_2:
            return SERCOM2_0_IRQn;
        case UART_3:
            return SERCOM3_0_IRQn;
        case UART_4:
            return SERCOM4_0_IRQn;
        case UART_5:
            return SERCOM5_0_IRQn;
        default:
            MBED_ASSERT(0);
    }
    return SERCOM0_0_IRQn; // to avoid warning
}

void serial_irq_set(serial_t *obj, SerialIrq irq, uint32_t enable)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);
    IRQn_Type irq_n = (IRQn_Type)0;
    irq_n = get_serial_irq_num(obj);
    uint8_t instance = serial_get_index(obj);

    if (enable) {
        switch (irq) {
            case RxIrq:
                _USART(obj).SERCOM_INTENSET = SERCOM_USART_INT_INTFLAG_RXC(1);
                irq_n = irq_n + 2;
                break;
            case TxIrq:
                _USART(obj).SERCOM_INTENSET = SERCOM_USART_INT_INTFLAG_TXC(1);
                irq_n = irq_n + 1;
                break;
        }

        _sercom_instances[instance] = obj;
        _sercom_set_handler(instance, uart_irq);

        NVIC_EnableIRQ(irq_n);

    } else {
        switch (irq) {
            case RxIrq:
                _USART(obj).SERCOM_INTENCLR = SERCOM_USART_INT_INTFLAG_RXC(1);
                irq_n = irq_n + 2;
                break;
            case TxIrq:
                _USART(obj).SERCOM_INTENCLR = SERCOM_USART_INT_INTFLAG_TXC(1);
                irq_n = irq_n + 1;
                break;
        }
        NVIC_DisableIRQ(irq_n);
    }
}

/******************************************************************************
 * READ/WRITE
 ******************************************************************************/
int serial_getc(serial_t *obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);
    while (!serial_readable(obj));
    return _USART(obj).SERCOM_DATA ;
}

void serial_putc(serial_t *obj, int c)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);
    uint32_t q = (c & SERCOM_USART_INT_DATA_Msk);
    while (!serial_writable(obj));
    _USART(obj).SERCOM_DATA = q;
    while (!(_USART(obj).SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_TXC(1)));  // wait till data is sent
}

int serial_readable(serial_t *obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);
    uint32_t status = 1;
    if (!(_USART(obj).SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_RXC(1))) {
        status = 0;
    } else {
        status = 1;
    }
    return status;
}

int serial_writable(serial_t *obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);
    uint32_t status = 1;
    if (!(_USART(obj).SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_DRE(1))) {
        status = 0;
    } else {
        status = 1;
    }
    return status;
}

const PinMap *serial_tx_pinmap()
{
    return PinMap_SERCOM;
}

const PinMap *serial_rx_pinmap()
{
    return PinMap_SERCOM;
}

const PinMap *serial_cts_pinmap()
{
    return PinMap_SERCOM;
}

const PinMap *serial_rts_pinmap()
{
    return PinMap_SERCOM;
}

/************************************************************************************
 * 			ASYNCHRONOUS HAL														*
 ************************************************************************************/

#if DEVICE_SERIAL_ASYNCH

/************************************
 * HELPER FUNCTIONS					*
 ***********************************/
void serial_tx_enable_event(serial_t *obj, int event, uint8_t enable)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);
    if(enable) {
        pSERIAL_S(obj)->events |= event;
    } else {
        pSERIAL_S(obj)->events &= ~ event;
    }
}

void serial_rx_enable_event(serial_t *obj, int event, uint8_t enable)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);
    if(enable) {
        pSERIAL_S(obj)->events |= event;
    } else {
        pSERIAL_S(obj)->events &= ~ event;
    }
}

void serial_tx_buffer_set(serial_t *obj, void *tx, int tx_length, uint8_t width)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);
    MBED_ASSERT(tx != (void*)0);
    // We only support byte buffers for now
    MBED_ASSERT(width == 8);

    if(serial_tx_active(obj)) return;

    obj->tx_buff.buffer = tx;
    obj->tx_buff.length = tx_length;
    obj->tx_buff.pos = 0;

    return;
}

void serial_rx_buffer_set(serial_t *obj, void *rx, int rx_length, uint8_t width)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);
    MBED_ASSERT(rx != (void*)0);
    // We only support byte buffers for now
    MBED_ASSERT(width == 8);

    if(serial_rx_active(obj)) return;

    obj->rx_buff.buffer = rx;
    obj->rx_buff.length = rx_length;
    obj->rx_buff.pos = 0;

    return;
}

void serial_set_char_match(serial_t *obj, uint8_t char_match)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);
    if (char_match != SERIAL_RESERVED_CHAR_MATCH) {
        obj->char_match = char_match;
    }
}

/************************************
 * TRANSFER FUNCTIONS				*
 ***********************************/
int serial_tx_asynch(serial_t *obj, const void *tx, size_t tx_length, uint8_t tx_width, uint32_t handler, uint32_t event, DMAUsage hint)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);
    MBED_ASSERT(tx != (void*)0);
    if(tx_length == 0) return 0;

    serial_tx_buffer_set(obj, (void *)tx, tx_length, tx_width);
    serial_tx_enable_event(obj, event, true);

    NVIC_ClearPendingIRQ(get_serial_irq_num(obj));
    NVIC_DisableIRQ(get_serial_irq_num(obj));
    NVIC_SetVector(get_serial_irq_num(obj), (uint32_t)handler);
    NVIC_EnableIRQ(get_serial_irq_num(obj));

    if (pUSART_S(obj)) {
        _USART(obj).SERCOM_INTENCLR = SERCOM_USART_INT_INTFLAG_TXC(1);
        _USART(obj).SERCOM_INTENSET = SERCOM_USART_INT_INTFLAG_DRE(1);
    }
    return 0;
}

void serial_rx_asynch(serial_t *obj, void *rx, size_t rx_length, uint8_t rx_width, uint32_t handler, uint32_t event, uint8_t char_match, DMAUsage hint)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);
    MBED_ASSERT(rx != (void*)0);

    serial_rx_enable_event(obj, SERIAL_EVENT_RX_ALL, false);
    serial_rx_enable_event(obj, event, true);
    serial_set_char_match(obj, char_match);
    serial_rx_buffer_set(obj, rx, rx_length, rx_width);

    NVIC_ClearPendingIRQ(get_serial_irq_num(obj));
    NVIC_DisableIRQ(get_serial_irq_num(obj));
    NVIC_SetVector(get_serial_irq_num(obj), (uint32_t)handler);
    NVIC_EnableIRQ(get_serial_irq_num(obj));

    if (pUSART_S(obj)) {
        _USART(obj).SERCOM_INTENSET = SERCOM_USART_INT_INTFLAG_RXC(1);
    }
    return;
}

uint8_t serial_tx_active(serial_t *obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);
    return ((obj->tx_buff.length > 0) ? true : false);
}

uint8_t serial_rx_active(serial_t *obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);
    return ((obj->rx_buff.length > 0) ? true : false);
}

int serial_tx_irq_handler_asynch(serial_t *obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);
    _USART(obj).SERCOM_INTENCLR = SERCOM_USART_INT_INTFLAG_TXC(1);
    serial_tx_abort_asynch(obj);
    return SERIAL_EVENT_TX_COMPLETE & obj->serial.events;
}

int serial_rx_irq_handler_asynch(serial_t *obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);
    int event = 0;
    /* This interrupt handler is called from USART irq */
    uint8_t *buf = (uint8_t*)obj->rx_buff.buffer;
    uint8_t error_code = 0;
    uint16_t received_data = 0;

    error_code = (uint8_t)(_USART(obj).SERCOM_STATUS & SERCOM_USART_INT_STATUS_Msk);
    /* Check if an error has occurred during the receiving */
    if (error_code) {
        /* Check which error occurred */
        if (error_code & SERCOM_USART_INT_STATUS_FERR(1)) {
            /* Store the error code and clear flag by writing 1 to it */
            _USART(obj).SERCOM_STATUS |= SERCOM_USART_INT_STATUS_FERR(1);
            serial_rx_abort_asynch(obj);
            return SERIAL_EVENT_RX_FRAMING_ERROR;
        } else if (error_code & SERCOM_USART_INT_STATUS_BUFOVF(1)) {
            /* Store the error code and clear flag by writing 1 to it */
            _USART(obj).SERCOM_STATUS |= SERCOM_USART_INT_STATUS_BUFOVF(1);
            serial_rx_abort_asynch(obj);
            return SERIAL_EVENT_RX_OVERFLOW;
        } else if (error_code & SERCOM_USART_INT_STATUS_PERR(1)) {
            /* Store the error code and clear flag by writing 1 to it */
            _USART(obj).SERCOM_STATUS |= SERCOM_USART_INT_STATUS_PERR(1);
            serial_rx_abort_asynch(obj);
            return SERIAL_EVENT_RX_PARITY_ERROR;
        }
    }

    /* Read current packet from DATA register,
    * increment buffer pointer and decrement buffer length */
    received_data = (_USART(obj).SERCOM_DATA & SERCOM_USART_INT_DATA_Msk);

    /* Read value will be at least 8-bits long */
    buf[obj->rx_buff.pos] = received_data;
    /* Increment 8-bit pointer */
    obj->rx_buff.pos++;

    /* Check if the last character have been received */
    if(--(obj->rx_buff.length) == 0) {
        event |= SERIAL_EVENT_RX_COMPLETE;
        if((buf[obj->rx_buff.pos - 1] == obj->char_match) && (obj->serial.events & SERIAL_EVENT_RX_CHARACTER_MATCH)) {
            event |= SERIAL_EVENT_RX_CHARACTER_MATCH;
        }
        _USART(obj).SERCOM_INTFLAG = SERCOM_USART_INT_INTFLAG_RXC(1);
        serial_rx_abort_asynch(obj);
        return event & obj->serial.events;
    }

    /* Check for character match event */
    if((buf[obj->rx_buff.pos - 1] == obj->char_match) && (obj->serial.events & SERIAL_EVENT_RX_CHARACTER_MATCH)) {
        event |= SERIAL_EVENT_RX_CHARACTER_MATCH;
    }

    /* Return to the call back if character match occured */
    if(event != 0) {
        serial_rx_abort_asynch(obj);
        return event & obj->serial.events;
    }
    return 0;
}

int serial_irq_handler_asynch(serial_t *obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);
    uint16_t interrupt_status;
    uint8_t *buf = obj->tx_buff.buffer;

    interrupt_status = _USART(obj).SERCOM_INTFLAG;
    interrupt_status &= _USART(obj).SERCOM_INTENSET;

    if (pUSART_S(obj)) {
        if (interrupt_status & SERCOM_USART_INT_INTFLAG_DRE(1)) {
            /* Interrupt has another TX source */
            if(obj->tx_buff.pos >= obj->tx_buff.length) {
                /* Transfer complete. Switch off interrupt and return event. */
                _USART(obj).SERCOM_INTENCLR = SERCOM_USART_INT_INTFLAG_DRE(1);
                _USART(obj).SERCOM_INTENSET = SERCOM_USART_INT_INTFLAG_TXC(1);
            } else {
                while((serial_writable(obj)) && (obj->tx_buff.pos <= (obj->tx_buff.length - 1))) {
                    _USART(obj).SERCOM_DATA = buf[obj->tx_buff.pos];
                    obj->tx_buff.pos++;
                }
            }
        }
        if (interrupt_status & SERCOM_USART_INT_INTFLAG_TXC(1)) {
            return serial_tx_irq_handler_asynch(obj);
        }
        if (interrupt_status & SERCOM_USART_INT_INTFLAG_RXC(1)) {
            return serial_rx_irq_handler_asynch(obj);
        }
    }
    return 0;
}

void serial_tx_abort_asynch(serial_t *obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);
    _USART(obj).SERCOM_INTFLAG = SERCOM_USART_INT_INTFLAG_TXC(1);
    _USART(obj).SERCOM_INTENCLR = SERCOM_USART_INT_INTFLAG_TXC(1);
    obj->tx_buff.length = 0;
    obj->rx_buff.pos = 0;

}

void serial_rx_abort_asynch(serial_t *obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);
    _USART(obj).SERCOM_INTFLAG = SERCOM_USART_INT_INTFLAG_RXC(1);
    _USART(obj).SERCOM_INTENCLR = SERCOM_USART_INT_INTFLAG_RXC(1);
    obj->rx_buff.length = 0;
    obj->rx_buff.pos = 0;
}

#endif
