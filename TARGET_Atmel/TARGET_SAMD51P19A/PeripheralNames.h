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
#ifndef MBED_PERIPHERALNAMES_H
#define MBED_PERIPHERALNAMES_H

#include "cmsis.h"
#include "PinNames.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    UART_0 = SERCOM0_BASE_ADDRESS,
    UART_1 = SERCOM1_BASE_ADDRESS,
    UART_2 = SERCOM2_BASE_ADDRESS,
    UART_3 = SERCOM3_BASE_ADDRESS,
    UART_4 = SERCOM4_BASE_ADDRESS,
    UART_5 = SERCOM5_BASE_ADDRESS,
    UART_6 = SERCOM6_BASE_ADDRESS,
    UART_7 = SERCOM7_BASE_ADDRESS,
} UARTName;

typedef enum {
    SERCOM_0,
    SERCOM_1,
    SERCOM_2,
    SERCOM_3,
    SERCOM_4,
    SERCOM_5,
    SERCOM_6,
    SERCOM_7,
} SERCOMName;

typedef enum {
    ADC_0,
    ADC_1,
} ADCName;

typedef enum {
    PWM_0 = TCC0_BASE_ADDRESS,
    PWM_1 = TCC1_BASE_ADDRESS,
    PWM_2 = TCC2_BASE_ADDRESS,
    PWM_3 = TCC3_BASE_ADDRESS,
    PWM_4 = TCC4_BASE_ADDRESS,
} PWMName;

typedef enum {
    EXTINT_0 = 0,
    EXTINT_1,
    EXTINT_2,
    EXTINT_3,
    EXTINT_4,
    EXTINT_5,
    EXTINT_6,
    EXTINT_7,
    EXTINT_8,
    EXTINT_9,
    EXTINT_10,
    EXTINT_11,
    EXTINT_12,
    EXTINT_13,
    EXTINT_14,
    EXTINT_15,
} EXTINTName;

#if defined(MBED_CONF_TARGET_STDIO_UART_TX)
#define STDIO_UART_TX     MBED_CONF_TARGET_STDIO_UART_TX
#else
#define STDIO_UART_TX     USBTX
#endif
#if defined(MBED_CONF_TARGET_STDIO_UART_RX)
#define STDIO_UART_RX     MBED_CONF_TARGET_STDIO_UART_RX
#else
#define STDIO_UART_RX     USBRX
#endif
#if defined(MBED_CONF_TARGET_STDIO_UART)
#define STDIO_UART        MBED_CONF_TARGET_STDIO_UART
#else
#define STDIO_UART        UART_2
#endif

#ifdef __cplusplus
}
#endif

#endif
