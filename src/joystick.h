#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <Arduino.h>
#include <atomic>

class Joystick {
public:
    enum Direction {
        CENTER, NORTH, SOUTH, EAST, WEST
    };

    Joystick(uint8_t xPin, uint8_t yPin);

    void update(uint8_t pressed);
    Direction get() const;
    bool pressed() const;

private:
    uint8_t _xPin;
    uint8_t _yPin;

    std::atomic<bool> _pressed{false};
    std::atomic<Direction> _dir{CENTER};

    static constexpr int DEADZONE = 100;

    void determineDirection(int x, int y);
};

#endif