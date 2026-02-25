#include "I2CBus.h"

#include <iostream>
#include <cstdlib>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>

I2CBus::I2CBus(const std::string& device) {
    fd_ = open(device.c_str(), O_RDWR);
    if (fd_ < 0) {
        std::cerr << "FATAL ERROR: Could not open " << device
                  << ". Check connections and permissions.\n";
        std::exit(EXIT_FAILURE);
    }
}

I2CBus::~I2CBus() {
    if (fd_ >= 0) close(fd_);
}

bool I2CBus::writeReg(uint8_t addr7, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    struct i2c_msg msg;
    msg.addr  = addr7;
    msg.flags = 0;
    msg.len   = static_cast<__u16>(sizeof(buf));
    msg.buf   = buf;

    struct i2c_rdwr_ioctl_data xfer;
    xfer.msgs  = &msg;
    xfer.nmsgs = 1;

    return ioctl(fd_, I2C_RDWR, &xfer) >= 0;
}

bool I2CBus::readRegs(uint8_t addr7, uint8_t start_reg, uint8_t* out, uint16_t n) {
    struct i2c_msg msgs[2];

    msgs[0].addr  = addr7;
    msgs[0].flags = 0;
    msgs[0].len   = 1;
    msgs[0].buf   = &start_reg;

    msgs[1].addr  = addr7;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len   = n;
    msgs[1].buf   = out;

    struct i2c_rdwr_ioctl_data xfer;
    xfer.msgs  = msgs;
    xfer.nmsgs = 2;

    return ioctl(fd_, I2C_RDWR, &xfer) >= 0;
}