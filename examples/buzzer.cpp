/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#if defined(MBED_BUZZER_EXAMPLE)
#include "mbed.h"
#include "SPI_TFT_ILI9341.h"
#include "Arial12x12.h"
#include "Arial28x28.h"
#include "GP5WaySwitch.h"

DigitalOut LCD_LED(PC05);
SPI_TFT_ILI9341 TFT(LCD_SPI_MOSI,
                    LCD_SPI_MISO,
                    LCD_SPI_SCK,
                    LCD_SPI_CS,
                    PC07, /* Reset */
                    PC06, /* DC */
                    "TFT");

PwmOut buzzer(PD11);

bool melody_on = false;

int scale(char s)
{
    int scale_table[] = {
        (int)(1000000.0 /  880.000), // A5
        (int)(1000000.0 /  987.767), // B5
        (int)(1000000.0 / 1046.502), // C6
        (int)(1000000.0 / 1174.659), // D6
        (int)(1000000.0 / 1318.510), // E6
        (int)(1000000.0 / 1396.913), // F6
        (int)(1000000.0 / 1567.982), // G6
    };

    return scale_table[s - 'A'];
}

void buzzer_melody()
{
    char melody[] = "CDE-CDE-GEDCDED-    ";
    static int code = -1;

    code = code + 1;
    if (melody[code] == ' ') {
        buzzer = 0.0;
        code = -1;
    } else if (melody[code] == '-') {
        // nothing to do
    } else {
        if (melody_on == true) {
            if (buzzer == 0.0)
                buzzer = 0.5;
            int period = scale(melody[code]);
            buzzer.period_us(period);
        }
    }
}

void gp5w_event(int event)
{
    if (event == GP5WaySwitch::EV_PRESS) {
        melody_on = !melody_on;
        if (melody_on == false) {
            buzzer = 0.0;
        }
    }
}

int main() {
    GP5WaySwitch gp5w(NULL);

    Ticker tsnd;
    tsnd.attach(buzzer_melody, 500ms);

    LCD_LED = 1;
    TFT.set_orientation(1);
    TFT.background(Black);
    TFT.foreground(White);
    TFT.cls();

    TFT.set_font((unsigned char *)Arial12x12);
    TFT.locate(10, 10);
    TFT.printf("Push Button");

    TFT.set_font((unsigned char *)Arial28x28);
    TFT.locate(130, 106);
    TFT.printf("OFF");

    while (true) {
        if (gp5w == GP5WaySwitch::EV_PRESS) {
            melody_on = !melody_on;
            TFT.locate(130, 106);
            if (melody_on == false) {
                buzzer = 0.0;
                TFT.foreground(White);
                TFT.printf("OFF");
            } else {
                TFT.foreground(Yellow);
                TFT.printf("ON ");
            }
            ThisThread::sleep_for(500ms);
        }
        ThisThread::sleep_for(1ms);
    }
}
#endif /* MBED_BUZZER_EXAMPLE */
