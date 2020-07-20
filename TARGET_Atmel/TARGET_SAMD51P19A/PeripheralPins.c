/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
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

#include "PeripheralPins.h"

/* PinMap.peripheral are:
 *   [32: 4] Peripheral Base Address if need
 *   [ 3: 0] Peripheral index
 * PinMap.function are:
 *   [23:16] PIN Number
 *   [ 7: 4] PAD Number if exsit
 *   [ 3: 0] MUX - Peripheral Multiplexing
 */

/************ADC***************/
#define ADC_AIN(n) ((n) << 4)

const PinMap PinMap_ADC[] = {
    //{PB03, ADC_7, NC},
    {PB08, ADC0_BASE_ADDRESS | ADC_0, PINMUX_PB08B_ADC0_AIN2 | ADC_AIN(2)},
    {PB09, ADC0_BASE_ADDRESS | ADC_0, PINMUX_PB09B_ADC0_AIN3 | ADC_AIN(3)},
    {PA07, ADC0_BASE_ADDRESS | ADC_0, PINMUX_PA07B_ADC0_AIN7 | ADC_AIN(7)},
    {PB04, ADC1_BASE_ADDRESS | ADC_1, PINMUX_PB04B_ADC1_AIN6 | ADC_AIN(6)},
    {PB05, ADC1_BASE_ADDRESS | ADC_1, PINMUX_PB05B_ADC1_AIN7 | ADC_AIN(7)},
    {PB06, ADC1_BASE_ADDRESS | ADC_1, PINMUX_PB06B_ADC1_AIN8 | ADC_AIN(8)},
    {PA04, ADC0_BASE_ADDRESS | ADC_0, PINMUX_PA04B_ADC0_AIN4 | ADC_AIN(4)},
    {PB07, ADC1_BASE_ADDRESS | ADC_1, PINMUX_PB07B_ADC1_AIN9 | ADC_AIN(9)},
    /* Not connected */
    {NC  , NC   , NC}
};

/************UART***************/
const PinMap PinMap_UART_TX[] = {
    //{PA03, UART_3, 0},
    /* Not connected */
    {NC  , NC   , NC}
};

const PinMap PinMap_UART_RX[] = {
    //{PA04, UART_3, 0},
    /* Not connected */
    {NC  , NC   , NC}
};

const PinMap PinMap_UART_CTS[] = {
    //{PA11, UART_5, 0},
    /* Not connected */
    {NC  , NC   , NC}
};

const PinMap PinMap_UART_RTS[] = {
    //{PA15, UART_2, 0},
    /* Not connected */
    {NC  , NC   , NC}
};


#define SERCOM_PAD(n) ((n) << 4)

const PinMap PinMap_SERCOM[] = {
    /* SERCOM0 */
    {PB24, SERCOM0_BASE_ADDRESS | SERCOM_0, PINMUX_PB24C_SERCOM0_PAD0 | SERCOM_PAD(0)},
    {PB25, SERCOM0_BASE_ADDRESS | SERCOM_0, PINMUX_PB25C_SERCOM0_PAD1 | SERCOM_PAD(1)},
    {PC24, SERCOM0_BASE_ADDRESS | SERCOM_0, PINMUX_PC24C_SERCOM0_PAD2 | SERCOM_PAD(2)},
    {PC25, SERCOM0_BASE_ADDRESS | SERCOM_0, PINMUX_PC25C_SERCOM0_PAD3 | SERCOM_PAD(3)},
    /* SERCOM1 */
    {PC22, SERCOM1_BASE_ADDRESS | SERCOM_1, PINMUX_PC22C_SERCOM1_PAD0 | SERCOM_PAD(0)},
    {PC23, SERCOM1_BASE_ADDRESS | SERCOM_1, PINMUX_PC23C_SERCOM1_PAD1 | SERCOM_PAD(1)},
    /* SERCOM2 */
    {PA12, SERCOM2_BASE_ADDRESS | SERCOM_2, PINMUX_PA12C_SERCOM2_PAD0 | SERCOM_PAD(0)},
    {PA13, SERCOM2_BASE_ADDRESS | SERCOM_2, PINMUX_PA13C_SERCOM2_PAD1 | SERCOM_PAD(1)},
    /* SERCOM3 */
    {PA17, SERCOM3_BASE_ADDRESS | SERCOM_3, PINMUX_PA17D_SERCOM3_PAD0 | SERCOM_PAD(0)},
    {PA16, SERCOM3_BASE_ADDRESS | SERCOM_3, PINMUX_PA16D_SERCOM3_PAD1 | SERCOM_PAD(1)},
    /* SERCOM4 */
    {PB08, SERCOM4_BASE_ADDRESS | SERCOM_4, PINMUX_PB08D_SERCOM4_PAD0 | SERCOM_PAD(0)},
    {PB09, SERCOM4_BASE_ADDRESS | SERCOM_4, PINMUX_PB09D_SERCOM4_PAD1 | SERCOM_PAD(1)},
    /* SERCOM5 */
    {PB02, SERCOM5_BASE_ADDRESS | SERCOM_5, PINMUX_PB02D_SERCOM5_PAD0 | SERCOM_PAD(0)},
    {PB03, SERCOM5_BASE_ADDRESS | SERCOM_5, PINMUX_PB03D_SERCOM5_PAD1 | SERCOM_PAD(1)},
    {PB00, SERCOM5_BASE_ADDRESS | SERCOM_5, PINMUX_PB00D_SERCOM5_PAD2 | SERCOM_PAD(2)},
    {PB01, SERCOM5_BASE_ADDRESS | SERCOM_5, PINMUX_PB01D_SERCOM5_PAD3 | SERCOM_PAD(3)},
    /* SERCOM6 */
    {PC16, SERCOM6_BASE_ADDRESS | SERCOM_6, PINMUX_PC16C_SERCOM6_PAD0 | SERCOM_PAD(0)},
    {PC17, SERCOM6_BASE_ADDRESS | SERCOM_6, PINMUX_PC17C_SERCOM6_PAD1 | SERCOM_PAD(1)},
    {PC18, SERCOM6_BASE_ADDRESS | SERCOM_6, PINMUX_PC18C_SERCOM6_PAD2 | SERCOM_PAD(2)},
    {PC19, SERCOM6_BASE_ADDRESS | SERCOM_6, PINMUX_PC19C_SERCOM6_PAD3 | SERCOM_PAD(3)},
    /* SERCOM7 */
    {PB21, SERCOM7_BASE_ADDRESS | SERCOM_7, PINMUX_PB21D_SERCOM7_PAD0 | SERCOM_PAD(0)},
    {PB20, SERCOM7_BASE_ADDRESS | SERCOM_7, PINMUX_PB20D_SERCOM7_PAD1 | SERCOM_PAD(1)},
    {PB18, SERCOM7_BASE_ADDRESS | SERCOM_7, PINMUX_PB18D_SERCOM7_PAD2 | SERCOM_PAD(2)},
    {PB19, SERCOM7_BASE_ADDRESS | SERCOM_7, PINMUX_PB19D_SERCOM7_PAD3 | SERCOM_PAD(3)},
    /* Not connected */
    {NC, NC, NC}
};

const PinMap PinMap_SERCOM_Ext[] = {
    //{PA08, SERCOM_2, 3},
    /* Not connected */
    {NC  , NC   , NC}
};

const PinMap PinMap_EXTINT[] = {
    /* EXTINT10 */
    {PC26, EXTINT_10, PINMUX_PC26A_EIC_EXTINT10},
    /* EXTINT11 */
    {PC27, EXTINT_11, PINMUX_PA27A_EIC_EXTINT11},
    /* EXTINT12 */
    {PC28, EXTINT_12, PINMUX_PA12A_EIC_EXTINT12},

    /* 5 Way Switch */
    {PD08, EXTINT_3, PINMUX_PD08A_EIC_EXTINT3},
    {PD09, EXTINT_4, PINMUX_PD09A_EIC_EXTINT4},
    {PD10, EXTINT_5, PINMUX_PD10A_EIC_EXTINT5},
    {PD12, EXTINT_7, PINMUX_PD12A_EIC_EXTINT7},
    {PD20, EXTINT_10, PINMUX_PD20A_EIC_EXTINT10},

    /* Not connected */
    {NC, NC, NC}
};

#define TC_TCC_WO(n) ((n) << 4)

const PinMap PinMap_PWM[] = {
    /* PWM0 */
    { PB08, TC4_BASE_ADDRESS, PINMUX_PB08E_TC4_WO0 | TC_TCC_WO(0) },
    /* PWM1 - Conflict with PWM4 */
    { PA07, TC1_BASE_ADDRESS, PINMUX_PA07E_TC1_WO1 | TC_TCC_WO(1) },
    /* PWM2 */
    { PB03, TC6_BASE_ADDRESS, PINMUX_PB03E_TC6_WO1 | TC_TCC_WO(1) },
    /* PWM3 */
    { PA04, TC0_BASE_ADDRESS, PINMUX_PA04E_TC0_WO0 | TC_TCC_WO(0) },
    /* PWM4 - Conflict with PWM1 */
    { PA06, TC1_BASE_ADDRESS, PINMUX_PA06E_TC1_WO0 | TC_TCC_WO(0) },

    /* Buzzer */
    { PD11, TCC0_BASE_ADDRESS, PINMUX_PD11F_TCC0_WO4 | TC_TCC_WO(4) },
    /* User LED */
    { PA15, TCC1_BASE_ADDRESS, PINMUX_PA15G_TCC1_WO3 | TC_TCC_WO(3) },

    /* Not connected */
    {NC  , NC   , NC}
};

