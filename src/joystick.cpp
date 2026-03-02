#include "Joystick.h"

Joystick::Joystick(uint8_t xPin, uint8_t yPin)
    : _xPin(xPin), _yPin(yPin) {}

void Joystick::update(uint8_t pressed) {
    int x = analogRead(_xPin);
    int y = analogRead(_yPin);

    _pressed.store(pressed, std::memory_order_relaxed);

    determineDirection(x, y);
}

Joystick::Direction Joystick::get() const {
    return _dir.load(std::memory_order_relaxed);
}

bool Joystick::pressed() const {
    return _pressed.load(std::memory_order_relaxed);
}

void Joystick::determineDirection(int x, int y) {
    int dx = x - 512;
    int dy = y - 512;

    Direction newDir;

    if (abs(dx) < DEADZONE && abs(dy) < DEADZONE)
        newDir = CENTER;
    else if (abs(dx) > abs(dy))
        newDir = (dx > 0) ? WEST : EAST;
    else
        newDir = (dy > 0) ? SOUTH : NORTH;

    _dir.store(newDir, std::memory_order_relaxed);
}