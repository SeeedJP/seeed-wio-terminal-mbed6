/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "example.h"
#if defined(MBED_ACCEL_EXAMPLE)
#include "mbed.h"
#include "SPI_TFT_ILI9341.h"
#include "Arial28x28.h"
#include "LIS3DHTR.h"

DigitalOut LCD_LED(PC05);
SPI_TFT_ILI9341 TFT(LCD_SPI_MOSI,
                    LCD_SPI_MISO,
                    LCD_SPI_SCK,
                    LCD_SPI_CS,
                    PC07, /* Reset */
                    PC06, /* DC */
                    "TFT");

LIS3DHTR lis(PA13, PA12);

static char *dtostrf(float val, int8_t width, uint8_t prec, char *sout)
{
    char *_buf = sout;
    if (val < 0) {
        *(_buf++) = '-';
        val *= -1;
    }
    int intp = (int)val;
    int decp = (int)((val - (float)intp) * pow(10, prec));
    int digit;
    for (digit = 0; ; digit++) {
        if (intp / (int)pow(10, digit) == 0) {
            break;
        }
    }
    if (digit == 0)
        digit++;
    for (int i = digit - 1; i >= 0; i--) {
        int n = intp / (int)pow(10, i);
        intp %= (int)pow(10, i);
        *(_buf++) = '0' + n;
    }
    if (prec > 0)
        *(_buf++) = '.';
    for (int i = prec - 1; i >= 0; i--) {
        int n = decp / (int)pow(10, i);
        decp %= (int)pow(10, i);
        *(_buf++) = '0' + n;
    }
    *(_buf++) = '\0';
    return sout;
}

int main()
{
    LCD_LED = 1;
    TFT.set_orientation(1);
    TFT.background(Black);
    TFT.foreground(White);
    TFT.cls();
    TFT.set_font((unsigned char *)Arial28x28);

    lis.setOutputDataRate(LIS3DHTR_DATARATE_50HZ);
    lis.setHighSolution(true);

    while (true) {
        char strX[16], strY[16], strZ[16];
        dtostrf(lis.getAccelerationX(), 2, 2, strX);
        dtostrf(lis.getAccelerationY(), 2, 2, strY);
        dtostrf(lis.getAccelerationZ(), 2, 2, strZ);
        TFT.locate(30, 106);
        TFT.printf("%s, %s, %s   ", strX, strY, strZ);
        ThisThread::sleep_for(500ms);
    }
}
#endif /* MBED_ACCEL_EXAMPLE */
