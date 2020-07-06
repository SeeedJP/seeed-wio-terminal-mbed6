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
#ifndef MBED_PINNAMES_H
#define MBED_PINNAMES_H

#include "cmsis.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PIN_INPUT,
    PIN_OUTPUT,
    PIN_INPUT_OUTPUT	//pin state can be set and read back
} PinDirection;

typedef enum {
    PA00  = 0,
    PA01  = 1,
    PA02  = 2,
    PA03  = 3,
    PA04  = 4,
    PA05  = 5,
    PA06  = 6,
    PA07  = 7,
    PA08  = 8,
    PA09  = 9,
    PA10  = 10,
    PA11  = 11,
    PA12  = 12,
    PA13  = 13,
    PA14  = 14,
    PA15  = 15,
    PA16  = 16,
    PA17  = 17,
    PA18  = 18,
    PA19  = 19,
    PA20  = 20,
    PA21  = 21,
    PA22  = 22,
    PA23  = 23,
    PA24  = 24,
    PA25  = 25,        
    //PA26  = 26,
    PA27  = 27,
    //PA28  = 28,
    //PA29  = 29,
    PA30  = 30,
    PA31  = 31,

    PB00  = 32,
    PB01  = 33,
    PB02  = 34,
    PB03  = 35,
    PB04  = 36,
    PB05  = 37,
    PB06  = 38,
    PB07  = 39,
    PB08  = 40,
    PB09  = 41,
    PB10  = 42,
    PB11  = 43,
    PB12  = 44,
    PB13  = 45,
    PB14  = 46,
    PB15  = 47,
    PB16  = 48,
    PB17  = 49,
    PB18  = 50,
    PB19  = 51,
    PB20  = 52,
    PB21  = 53,
    PB22  = 54,
    PB23  = 55,
    PB24  = 56,
    PB25  = 57,
    PB26  = 58,
    PB27  = 59,
    PB28  = 60,
    PB29  = 61,
    PB30  = 62,
    PB31  = 63,

    PC00  = 64,
    PC01  = 65,
    PC02  = 66,
    PC03  = 67,
    PC04  = 68,
    PC05  = 69,
    PC06  = 70,
    PC07  = 71,
    //PC08  = 72,
    //PC09  = 73,
    PC10  = 74,
    PC11  = 75,
    PC12  = 76,
    PC13  = 77,
    PC14  = 78,
    PC15  = 79,
    PC16  = 80,
    PC17  = 81,
    PC18  = 82,
    PC19  = 83,
    PC20  = 84,
    PC21  = 85,
    PC22  = 86,
    PC23  = 87,
    PC24  = 88,
    PC25  = 89,
    PC26  = 90,
    PC27  = 91,
    PC28  = 92,
    //PC29  = 93,
    PC30  = 94,
    PC31  = 95,

    PD00  = 96,
    PD01  = 97,
    //PD02  = 98,
    //PD03  = 99,
    //PD04  = 100,
    //PD05  = 101,
    //PD06  = 102,
    //PD07  = 103,
    PD08  = 104,
    PD09  = 105,
    PD10  = 106,
    PD11  = 107,
    PD12  = 108,
    //PD13  = 109,
    //PD14  = 110,
    //PD15  = 111,
    //PD16  = 112,
    //PD17  = 113,
    //PD18  = 114,
    //PD19  = 115,
    PD20  = 116,
    PD21  = 117,
    //PD22  = 118,
    //PD23  = 119,
    //PD24  = 120,
    //PD25  = 121,
    //PD26  = 122,
    //PD27  = 123,
    //PD28  = 124,
    //PD29  = 125,
    //PD30  = 126,
    //PD31  = 127,

    // Not connected
    NC = (int)0xFFFFFFFF,

    USBTX = PA25,
    USBRX = PA24,

    LED1 = PA15,
    LED2 = PA15,
    LED3 = PA15,
    LED4 = PA15,

    BUTTON1 = PC26,
    BUTTON2 = PC27,
    BUTTON3 = PC28,

    RTL8720D_SPI_MOSI = PB24,
    RTL8720D_SPI_MISO = PC24,
    RTL8720D_SPI_SCK  = PB25,
    RTL8720D_SPI_CS   = PC25,

    LCD_SPI_MOSI = PB19,
    LCD_SPI_MISO = PB18,
    LCD_SPI_SCK  = PB20,
    LCD_SPI_CS   = PB21,

    SDCARD_SPI_MOSI = PC16,
    SDCARD_SPI_MISO = PC18,
    SDCARD_SPI_SCK  = PC17,
    SDCARD_SPI_CS   = PC19,

    RPI_SPI_MOSI = PB02,
    RPI_SPI_MISO = PB00,
    RPI_SPI_SCK  = PB03,
    RPI_SPI_CS   = PB01,

    SPI_MOSI = NC,
    SPI_MISO = NC,
    SPI_SCK  = NC,
    SPI_CS   = NC,
} PinName;

typedef enum {
    PullNone = 0,
    PullUp = 1,
    PullDown = 2,
    PullDefault = PullUp
} PinMode;

#ifdef __cplusplus
}
#endif

#endif
