/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#if defined(MBED_LIGHT_EXAMPLE)
#include "mbed.h"
#include "SPI_TFT_ILI9341.h"
#include "Arial28x28.h"

DigitalOut LCD_LED(PC05);
SPI_TFT_ILI9341 TFT(LCD_SPI_MOSI,
                    LCD_SPI_MISO,
                    LCD_SPI_SCK,
                    LCD_SPI_CS,
                    PC07, /* Reset */
                    PC06, /* DC */
                    "TFT");

AnalogIn light(PD01);

int main()
{
    LCD_LED = 1;
    TFT.set_orientation(1);
    TFT.background(Black);
    TFT.foreground(White);
    TFT.cls();
    TFT.set_font((unsigned char *)Arial28x28);

    while (true) {
        TFT.locate(120, 106);
        TFT.printf("%d%%  ", (int)(light * 100));
        ThisThread::sleep_for(500ms);
    }
}
#endif /* MBED_LIGHT_EXAMPLE */
