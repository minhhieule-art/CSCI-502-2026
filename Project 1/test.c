// minimu9v5_lsm6ds33_accel.c
// Build: gcc -O2 -Wall minimu9v5_lsm6ds33_accel.c -o accel
// Run:   sudo ./accel

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

// --- Minimal I2C_RDWR definitions (avoid external headers) ---
#ifndef I2C_M_RD
#define I2C_M_RD 0x0001
#endif

#ifndef I2C_RDWR
#define I2C_RDWR 0x0707
#endif

struct i2c_msg {
    uint16_t addr;
    uint16_t flags;
    uint16_t len;
    uint8_t *buf;
};

struct i2c_rdwr_ioctl_data {
    struct i2c_msg *msgs;
    uint32_t nmsgs;
};

// --- LSM6DS33 (MinIMU-9 v5) ---
#define I2C_DEV        "/dev/i2c-1"
#define LSM6_ADDR      0x6B   // default when SA0 is high on MinIMU-9 v5

#define REG_CTRL1_XL   0x10
#define REG_CTRL3_C    0x12
#define REG_OUTX_L_XL  0x28   // start of accel output (X_L, X_H, Y_L, Y_H, Z_L, Z_H)

static int i2c_write_reg(int fd, uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    struct i2c_msg msg = { addr, 0, 2, buf };
    struct i2c_rdwr_ioctl_data xfer = { &msg, 1 };
    return ioctl(fd, I2C_RDWR, &xfer);
}

static int i2c_read_regs(int fd, uint8_t addr, uint8_t start_reg, uint8_t *out, uint16_t len)
{
    uint8_t regbuf[1] = { start_reg };
    struct i2c_msg msgs[2] = {
        { addr, 0,        1,   regbuf }, // write register pointer
        { addr, I2C_M_RD, len, out    }  // read data
    };
    struct i2c_rdwr_ioctl_data xfer = { msgs, 2 };
    return ioctl(fd, I2C_RDWR, &xfer);
}

static int16_t u8_to_i16(uint8_t lo, uint8_t hi)
{
    return (int16_t)((hi << 8) | lo);
}

int main(void)
{
    int fd = open(I2C_DEV, O_RDWR);
    if (fd < 0) { perror("open /dev/i2c-1"); return 1; }

    // CTRL3_C: BDU=1 (bit6), IF_INC=1 (bit2) => 0x44
    if (i2c_write_reg(fd, LSM6_ADDR, REG_CTRL3_C, 0x44) < 0) {
        perror("write CTRL3_C"); close(fd); return 1;
    }

    // CTRL1_XL: ODR=104 Hz, FS=±2g, BW=400 Hz => 0x40 (good starter setting)
    if (i2c_write_reg(fd, LSM6_ADDR, REG_CTRL1_XL, 0x40) < 0) {
        perror("write CTRL1_XL"); close(fd); return 1;
    }

    // Sensitivity for ±2g: 0.061 mg/LSB = 0.000061 g/LSB
    const float g_per_lsb = 0.000061f;

    while (1) {
        uint8_t raw[6];
        if (i2c_read_regs(fd, LSM6_ADDR, REG_OUTX_L_XL, raw, 6) < 0) {
            perror("read accel"); close(fd); return 1;
        }

        int16_t ax = u8_to_i16(raw[0], raw[1]);
        int16_t ay = u8_to_i16(raw[2], raw[3]);
        int16_t az = u8_to_i16(raw[4], raw[5]);

        printf("raw: ax=%6d ay=%6d az=%6d | g: ax=%+.4f ay=%+.4f az=%+.4f\n",
               ax, ay, az,
               ax * g_per_lsb, ay * g_per_lsb, az * g_per_lsb);

        usleep(100000); // 100 ms
    }

    close(fd);
    return 0;
}
