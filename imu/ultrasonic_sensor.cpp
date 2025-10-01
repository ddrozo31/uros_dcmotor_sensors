#include "ultrasonic_sensor.hpp"

UltrasonicSensor::UltrasonicSensor(uint trig_pin, uint echo_pin)
    : trig(trig_pin), echo(echo_pin) {
    gpio_init(trig);
    gpio_set_dir(trig, GPIO_OUT);
    gpio_put(trig, 0);

    gpio_init(echo);
    gpio_set_dir(echo, GPIO_IN);
}

bool UltrasonicSensor::measure_distance(float& distance_meters) {
    // Send 10µs pulse to trigger
    gpio_put(trig, 1);
    sleep_us(10);
    gpio_put(trig, 0);

    // Wait for echo to go high
    absolute_time_t start_time = get_absolute_time();
    while (!gpio_get(echo)) {
        if (absolute_time_diff_us(start_time, get_absolute_time()) > 100000) return false;
    }

    // Measure how long echo stays high
    absolute_time_t echo_start = get_absolute_time();
    while (gpio_get(echo)) {
        if (absolute_time_diff_us(echo_start, get_absolute_time()) > 30000) return false;
    }
    absolute_time_t echo_end = get_absolute_time();

    int64_t pulse_width_us = absolute_time_diff_us(echo_start, echo_end);
    distance_meters = (pulse_width_us / 2.0f) * 0.000343f;

    return true;
}
