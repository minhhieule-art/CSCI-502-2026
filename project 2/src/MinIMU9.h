#pragma once
#include "CommInterface.h"

#include <cstdint>

class MinIMU9 {
public:
    explicit MinIMU9(CommInterface* commBus);

    void init();

    void readData(float& ax, float& ay, float& az,
                  float& gx, float& gy, float& gz,
                  float& mx, float& my, float& mz);

private:
    CommInterface* bus_ = nullptr;

    static constexpr uint8_t ADDR_LSM6 = 0x6B;
    static constexpr uint8_t ADDR_LIS3 = 0x1E;

    // Scale factors for chosen full-scale settings
    static constexpr float acc_g_per_lsb      = 0.000061f;   // +/- 2g
    static constexpr float gyr_dps_per_lsb    = 0.00875f;     // +/- 245 dps
    static constexpr float mag_gauss_per_lsb  = 1.0f / 6842.0f; // +/- 4 gauss

    static int16_t le16_to_i16(uint8_t lo, uint8_t hi);
};