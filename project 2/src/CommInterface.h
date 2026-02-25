#pragma once
#include <cstdint>

class CommInterface {
public:
    virtual ~CommInterface() = default;
    virtual bool writeReg(uint8_t addr7, uint8_t reg, uint8_t val) = 0;
    virtual bool readRegs(uint8_t addr7, uint8_t start_reg, uint8_t* out, uint16_t n) = 0;
};