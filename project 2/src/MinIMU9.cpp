#include "MinIMU9.h"

#include <iostream>
#include <cstdlib>
#include <cmath>

MinIMU9::MinIMU9(CommInterface* commBus) : bus_(commBus) {}

int16_t MinIMU9::le16_to_i16(uint8_t lo, uint8_t hi) {
    return static_cast<int16_t>(static_cast<uint16_t>(lo) |
                                (static_cast<uint16_t>(hi) << 8));
}

void MinIMU9::init() {
    // LSM6DS33 Config
    if (!bus_->writeReg(ADDR_LSM6, 0x12, 0x44) || // CTRL3_C (BDU, IF_INC)
        !bus_->writeReg(ADDR_LSM6, 0x10, 0x40) || // CTRL1_XL (Accel 104Hz, 2g)
        !bus_->writeReg(ADDR_LSM6, 0x11, 0x40)) { // CTRL2_G (Gyro 104Hz, 245dps)
        std::cerr << "Failed to initialize LSM6DS33.\n";
        std::exit(EXIT_FAILURE);
    }

    // LIS3MDL Config
    if (!bus_->writeReg(ADDR_LIS3, 0x20, 0x70) || // CTRL_REG1 (XY Ultra-high, 10Hz)
        !bus_->writeReg(ADDR_LIS3, 0x21, 0x00) || // CTRL_REG2 (+/- 4 gauss)
        !bus_->writeReg(ADDR_LIS3, 0x22, 0x00) || // CTRL_REG3 (Continuous mode)
        !bus_->writeReg(ADDR_LIS3, 0x23, 0x0C) || // CTRL_REG4 (Z Ultra-high)
        !bus_->writeReg(ADDR_LIS3, 0x24, 0x40)) { // CTRL_REG5 (BDU)
        std::cerr << "Failed to initialize LIS3MDL.\n";
        std::exit(EXIT_FAILURE);
    }

    std::cout << "Hardware initialized successfully.\n";
}

void MinIMU9::readData(float& ax, float& ay, float& az,
                       float& gx, float& gy, float& gz,
                       float& mx, float& my, float& mz) {
    uint8_t gbuf[6], abuf[6], mbuf[6];

    if (!bus_->readRegs(ADDR_LSM6, 0x22, gbuf, 6) ||
        !bus_->readRegs(ADDR_LSM6, 0x28, abuf, 6) ||
        !bus_->readRegs(ADDR_LIS3, static_cast<uint8_t>(0x28 | 0x80), mbuf, 6)) {
        std::cerr << "Hardware Read Error!\n";
        std::exit(EXIT_FAILURE);
    }

    // Convert gyro to rad/s
    const float dps_to_rads = static_cast<float>(M_PI / 180.0);

    gx = le16_to_i16(gbuf[0], gbuf[1]) * (gyr_dps_per_lsb * dps_to_rads);
    gy = le16_to_i16(gbuf[2], gbuf[3]) * (gyr_dps_per_lsb * dps_to_rads);
    gz = le16_to_i16(gbuf[4], gbuf[5]) * (gyr_dps_per_lsb * dps_to_rads);

    ax = le16_to_i16(abuf[0], abuf[1]) * acc_g_per_lsb;
    ay = le16_to_i16(abuf[2], abuf[3]) * acc_g_per_lsb;
    az = le16_to_i16(abuf[4], abuf[5]) * acc_g_per_lsb;

    mx = le16_to_i16(mbuf[0], mbuf[1]) * mag_gauss_per_lsb;
    my = le16_to_i16(mbuf[2], mbuf[3]) * mag_gauss_per_lsb;
    mz = le16_to_i16(mbuf[4], mbuf[5]) * mag_gauss_per_lsb;
}