#include "mbed.h"
#include "GP5WaySwitch.h"

GP5WaySwitch::GP5WaySwitch(Callback <void (int)> eventHandler, PinName pX, PinName pY, PinName pZ, PinName pB, PinName pU)
    : x(pX), y(pY), z(pZ), b(pB), u(pU)
{
    _eventHandler = eventHandler;
    if (_eventHandler) {
        _preX = x.read();
        _preY = y.read();
        _preZ = z.read();
        _preB = b.read();
        _preU = u.read();
        t.attach(callback(this, &GP5WaySwitch::_handler), 33ms);
    }
}

GP5WaySwitch::~GP5WaySwitch()
{
    if (_eventHandler) {
        t.detach();
    }
}

void GP5WaySwitch::_handler()
{
    int _x = x.read();
    int _y = y.read();
    int _z = z.read();
    int _b = b.read();
    int _u = u.read();

    if (_preX != _x && _x == 0) {
        _eventHandler(EV_DOWN);
    } else if (_preY != _y && _y == 0) {
        _eventHandler(EV_RIGHT);
    } else if (_preZ != _z && _z == 0) {
        _eventHandler(EV_PRESS);
    } else if (_preB != _b && _b == 0) {
        _eventHandler(EV_LEFT);
    } else if (_preU != _u && _u == 0) {
        _eventHandler(EV_UP);
    }
    _preX = _x;
    _preY = _y;
    _preZ = _z;
    _preB = _b;
    _preU = _u;
}

GP5WaySwitch::operator int()
{
    if (!x.read()) return EV_DOWN;
    if (!y.read()) return EV_RIGHT;
    if (!b.read()) return EV_LEFT;
    if (!u.read()) return EV_UP;
    if (!z.read()) return EV_PRESS;
    return EV_NONE;
}
