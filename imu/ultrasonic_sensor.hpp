#ifndef ULTRASONIC_SENSOR_HPP
#define ULTRASONIC_SENSOR_HPP

#include "pico/stdlib.h"

class UltrasonicSensor {
public:
    UltrasonicSensor(uint trig_pin, uint echo_pin);
    bool measure_distance(float& distance_meters);

private:
    uint trig;
    uint echo;
};

#endif
