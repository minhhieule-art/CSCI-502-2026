#pragma once
#include "CommInterface.h"

#include <string>
#include <cstdint>

class I2CBus : public CommInterface {
public:
    explicit I2CBus(const std::string& device);
    ~I2CBus() override;

    I2CBus(const I2CBus&) = delete;
    I2CBus& operator=(const I2CBus&) = delete;

    bool writeReg(uint8_t addr7, uint8_t reg, uint8_t val) override;
    bool readRegs(uint8_t addr7, uint8_t start_reg, uint8_t* out, uint16_t n) override;

private:
    int fd_ = -1;
};