/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"

// Blinking rate in milliseconds
#define BLINKING_RATE_MS     (1000)

UnbufferedSerial uart4(PB08, PB09, 115200);
BufferedSerial uart1(PA16, PA17, 115200);

void __button_handler(const char *str)
{
    uart4.write(str, strlen(str));
}
void button1_handler(void) { __button_handler("Button1 Rise\r\n"); }
void button2_handler(void) { __button_handler("Button2 Fall\r\n"); }
void button3_rise_handler(void) { __button_handler("Button3 Rise\r\n"); }
void button3_fall_handler(void) { __button_handler("Button3 Fall\r\n"); }

int main()
{
    DigitalOut led(LED1);
    InterruptIn button1(BUTTON1);
    InterruptIn button2(BUTTON2);
    InterruptIn button3(BUTTON3);
    button1.rise(&button1_handler);
    button2.fall(&button2_handler);
    button3.rise(&button3_rise_handler);
    button3.fall(&button3_fall_handler);

    while (true) {
        led = !led;
        uart4.write("UART4: Hello Serial4\r\n", 22);
        uart1.write("UART1: Hello Serial1\r\n", 22);
#if 0
        printf("Mbed OS version %d.%d.%d\r\n", MBED_MAJOR_VERSION, MBED_MINOR_VERSION, MBED_PATCH_VERSION);
#endif

        thread_sleep_for(BLINKING_RATE_MS);
    }
}
