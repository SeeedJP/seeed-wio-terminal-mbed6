/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

/* Select one from list below */
#define MBED_BLINKY_EXAMPLE
//#define MBED_LCD_EXAMPLE
//#define MBED_BUZZER_EXAMPLE
//#define MBED_LIGHT_EXAMPLE
//#define MBED_ACCEL_EXAMPLE

#if defined(MBED_LCD_EXAMPLE)
#include "examples/lcd.cpp"
#elif defined(MBED_BUZZER_EXAMPLE)
#include "examples/buzzer.cpp"
#elif defined(MBED_LIGHT_EXAMPLE)
#include "examples/light.cpp"
#elif defined(MBED_ACCEL_EXAMPLE)
#include "examples/accel.cpp"
#else /* MBED_BLINKY_EXAMPLE */
#include "mbed.h"

/* Blinking rate in milliseconds */
#define BLINKING_RATE     (500ms)

int main()
{
    /* Initialise the digital pin LED1 as an output */
    DigitalOut led(LED1);

    printf("hello, Mbed OS 6\n");
    while (true) {
        led = !led;
        ThisThread::sleep_for(BLINKING_RATE);
    }
}
#endif /* MBED_BLINKEY_EXAMPLE */
