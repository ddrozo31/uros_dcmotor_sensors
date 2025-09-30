// imu/madgwick_filter.hpp
#ifndef MADGWICK_FILTER_HPP
#define MADGWICK_FILTER_HPP

#include <cmath>

class MadgwickFilter {
public:
    float q0, q1, q2, q3; // quaternion
    float beta;           // algorithm gain

    MadgwickFilter(float beta = 0.1f) : q0(1), q1(0), q2(0), q3(0), beta(beta) {}

    void update(float gx, float gy, float gz,
                float ax, float ay, float az, float dt);

    void getQuaternion(float& x, float& y, float& z, float& w) {
        x = q1;
        y = q2;
        z = q3;
        w = q0;
    }
};

#endif
