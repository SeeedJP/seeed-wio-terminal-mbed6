/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "example.h"
#if defined(MBED_BLINKY_EXAMPLE)
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
