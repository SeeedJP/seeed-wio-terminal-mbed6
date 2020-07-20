#ifndef GP5WAYSWITCH_H
#define GP5WAYSWITCH_H

#include "mbed.h"

/**
 * Example code: with use eventHandler
 *   if eventHandler is set, it will call by any event happen.
 *

#include "GP5WaySwitch.h"

DigitalOut led(LED1);
void gp5w_event(int event)
{
    if (event == GP5WaySwitch::EV_UP) {
        led = !led;
    }
    if (event == GP5WaySwitch::EV_DOWN) {
    }
    if (event == GP5WaySwitch::EV_LEFT) {
    }
    if (event == GP5WaySwitch::EV_RIGHT) {
    }
    if (event == GP5WaySwitch::EV_PRESS) {
    }
}
GP5WaySwitch gp5w(gp5w_event);

void main()
{
    while (true) {
        thread_sleep_for(1);
    }
}
 */

/**
 * Example code: without use eventHandler

#include "GP5WaySwitch.h"

void main()
{
    GP5WaySwitch gp5w(NULL);
    while (true) {
        switch (gp5w) {
            case gp5w.EV_UP:    printf("EV_UP\n");    break;
            case gp5w.EV_DOWN:  printf("EV_DOWN\n");  break;
            case gp5w.EV_LEFT:  printf("EV_LEFT\n");  break;
            case gp5w.EV_RIGHT: printf("EV_RIGHT\n"); break;
            case gp5w.EV_PRESS: printf("EV_PRESS\n"); break;
            default:                                  break;
        }
        thread_sleep_for(1);
    }
}
 */

class GP5WaySwitch {
public:
    GP5WaySwitch(Callback <void (int)> eventHandler = NULL,
                 PinName pX = PD08,
                 PinName pY = PD09,
                 PinName pZ = PD10,
                 PinName pB = PD12,
                 PinName pU = PD20);

    ~GP5WaySwitch();

    enum gp5wEvent {
        EV_NONE,
        EV_UP,
        EV_DOWN,
        EV_LEFT,
        EV_RIGHT,
        EV_PRESS,
    };

    operator int();

public:
    DigitalIn x;
    DigitalIn y;
    DigitalIn z;
    DigitalIn b;
    DigitalIn u;

private:
    int _preX;
    int _preY;
    int _preZ;
    int _preB;
    int _preU;

    Ticker t;
    Callback<void (int)> _eventHandler;
    void _handler();
};

#endif
