#pragma once

class MadgwickFilter {
public:
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    float beta = 0.1f;

    void update(float gx, float gy, float gz,
                float ax, float ay, float az,
                float mx, float my, float mz,
                float dt);
};