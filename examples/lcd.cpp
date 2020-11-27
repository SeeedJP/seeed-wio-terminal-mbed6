/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "example.h"
#if defined(MBED_LCD_EXAMPLE)
#include "mbed.h"
#include "SPI_TFT_ILI9341.h"
#include "Arial12x12.h"
#include "Arial24x23.h"
#include "Arial28x28.h"
#include "font_big.h"

DigitalOut LCD_LED(PC05);
SPI_TFT_ILI9341 TFT(LCD_SPI_MOSI,
                    LCD_SPI_MISO,
                    LCD_SPI_SCK,
                    LCD_SPI_CS,
                    PC07, /* Reset */
                    PC06, /* DC */
                    "TFT");

void sample1() {
    /* first show the 4 directions */
    TFT.background(Black);
    TFT.cls();

    TFT.set_font((unsigned char *)Arial12x12);
    TFT.set_orientation(0);
    TFT.locate(0, 0);
    printf("  Hello Mbed 0");
    TFT.set_orientation(1);
    TFT.locate(0, 0);
    printf("  Hello Mbed 1");
    TFT.set_orientation(2);
    TFT.locate(0, 0);
    printf("  Hello Mbed 2");
    TFT.set_orientation(3);
    TFT.locate(0, 0);
    printf("  Hello Mbed 3");
    TFT.set_orientation(1);
    TFT.set_font((unsigned char *)Arial24x23);
    TFT.locate(50, 100);
    TFT.printf("TFT orientation");
    TFT.set_font((unsigned char *)Arial12x12);
}

void sample2() {
    /* draw some graphics */
    TFT.set_orientation(1);
    TFT.background(Black);

    TFT.cls();
    TFT.set_font((unsigned char *)Arial24x23);
    TFT.locate(100, 100);
    TFT.printf("Graphic");

    TFT.line(0, 0, 100, 0, Green);
    TFT.line(0, 0, 0, 200, Green);
    TFT.line(0, 0, 100, 200, Green);

    TFT.rect(100, 50, 150, 100, Red);
    TFT.fillrect(180, 25, 220, 70, Blue);

    TFT.circle(80, 150, 33, White);
    TFT.fillcircle(160, 190, 20, Yellow);

    double s;
    for (int i = 0; i < 320; i++) {
        s = 20 * sin((long double)i / 10);
        TFT.pixel(i, 100 + (int)s, Red);
    }
}

void sample3() {
    /* bigger text */
    TFT.foreground(White);
    TFT.background(Blue);
    TFT.set_orientation(1);
    TFT.cls();
    TFT.set_font((unsigned char *)Arial24x23);
    TFT.locate(0, 0);
    TFT.printf("Different Fonts :");

    TFT.set_font((unsigned char *)Neu42x35);
    TFT.locate(0, 30);
    TFT.printf("Hello Mbed 1");
    TFT.set_font((unsigned char *)Arial24x23);
    TFT.locate(20, 80);
    TFT.printf("Hello Mbed 2");
    TFT.set_font((unsigned char *)Arial12x12);
    TFT.locate(35, 120);
    TFT.printf("Hello Mbed 3");
}

enum {
    NONE,
    SAMPLE1,
    SAMPLE2,
    SAMPLE3,
};

static int event = NONE;
void button1_handler(void) { event = SAMPLE1; }
void button2_handler(void) { event = SAMPLE2; }
void button3_handler(void) { event = SAMPLE3; }

int main()
{
    InterruptIn button1(BUTTON1);
    InterruptIn button2(BUTTON2);
    InterruptIn button3(BUTTON3);
    button1.rise(&button1_handler);
    button2.rise(&button2_handler);
    button3.rise(&button3_handler);

    /* backlight on */
    LCD_LED = 1;

    TFT.claim(stdout);
    TFT.set_orientation(1);
    TFT.background(Black);
    TFT.foreground(White);
    TFT.cls();

    TFT.set_font((unsigned char *)Arial12x12);
    TFT.locate(160, 200);
    TFT.printf("Push Any Button");

    while (true) {
        switch (event) {
            case SAMPLE1:
                sample1();
                break;
            case SAMPLE2:
                sample2();
                break;
            case SAMPLE3:
                sample3();
                break;
        }

        if (event != NONE)
            event = NONE;

        ThisThread::sleep_for(1ms);
    }
}
#endif /* MBED_LCD_EXAMPLE */
