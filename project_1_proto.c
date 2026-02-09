// minimu9v5_read.c
// Read Pololu MinIMU-9 v5 (LSM6DS33 accel+gyro + LIS3MDL magnetometer)
// using only /dev/i2c-* (no external libraries).

#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define I2C_DEV "/dev/i2c-1"

// MinIMU-9 v5 default addresses (SA0 high) :contentReference[oaicite:14]{index=14}
#define ADDR_LSM6DS33 0x6B
#define ADDR_LIS3MDL  0x1E

// --- LSM6DS33 registers ---
#define LSM6_WHO_AM_I   0x0F
#define LSM6_CTRL1_XL   0x10
#define LSM6_CTRL2_G    0x11
#define LSM6_CTRL3_C    0x12
#define LSM6_OUTX_L_G   0x22  // gyro x LSB :contentReference[oaicite:15]{index=15}
#define LSM6_OUTX_L_XL  0x28  // accel x LSB :contentReference[oaicite:16]{index=16}

// --- LIS3MDL registers ---
#define LIS_WHO_AM_I    0x0F  // returns 0x3D (00111101b) :contentReference[oaicite:17]{index=17}
#define LIS_CTRL_REG1   0x20
#define LIS_CTRL_REG2   0x21
#define LIS_CTRL_REG3   0x22
#define LIS_CTRL_REG4   0x23
#define LIS_CTRL_REG5   0x24
#define LIS_OUT_X_L     0x28  // mag x LSB :contentReference[oaicite:18]{index=18}

// Low-level: combined write(reg) then read(N) using repeated-start (I2C_RDWR)
static int i2c_write_reg(int fd, uint8_t addr7, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    struct i2c_msg msg = {
        .addr  = addr7,
        .flags = 0,
        .len   = (uint16_t)sizeof(buf),
        .buf   = buf
    };
    struct i2c_rdwr_ioctl_data xfer = { .msgs = &msg, .nmsgs = 1 };

    if (ioctl(fd, I2C_RDWR, &xfer) < 0) return -1;
    return 0;
}

static int i2c_read_regs(int fd, uint8_t addr7, uint8_t start_reg, uint8_t *out, uint16_t n)
{
    struct i2c_msg msgs[2];

    msgs[0].addr  = addr7;
    msgs[0].flags = 0;          // write
    msgs[0].len   = 1;
    msgs[0].buf   = &start_reg;

    msgs[1].addr  = addr7;
    msgs[1].flags = I2C_M_RD;   // read
    msgs[1].len   = n;
    msgs[1].buf   = out;

    struct i2c_rdwr_ioctl_data xfer = { .msgs = msgs, .nmsgs = 2 };
    if (ioctl(fd, I2C_RDWR, &xfer) < 0) return -1;
    return 0;
}

static int16_t le16_to_i16(uint8_t lo, uint8_t hi)
{
    return (int16_t)((uint16_t)lo | ((uint16_t)hi << 8));
}

static void die(const char *msg)
{
    fprintf(stderr, "%s: %s\n", msg, strerror(errno));
}

int main(void)
{
    int fd = open(I2C_DEV, O_RDWR);
    if (fd < 0) { die("open(/dev/i2c-1) failed"); return 1; }

    // ---------- Identify LSM6DS33 ----------
    uint8_t who = 0;
    if (i2c_read_regs(fd, ADDR_LSM6DS33, LSM6_WHO_AM_I, &who, 1) < 0) {
        die("LSM6DS33 WHO_AM_I read failed"); close(fd); return 1;
    }
    // LSM6DS33 WHO_AM_I is fixed at 0x69 (69h). :contentReference[oaicite:19]{index=19}
    printf("LSM6DS33 WHO_AM_I = 0x%02X (expected 0x69)\n", who);

    // ---------- Configure LSM6DS33 ----------
    // CTRL3_C: set BDU=1 (bit6) and IF_INC=1 (bit2) => 0x44. :contentReference[oaicite:20]{index=20}
    if (i2c_write_reg(fd, ADDR_LSM6DS33, LSM6_CTRL3_C, 0x44) < 0) {
        die("LSM6DS33 write CTRL3_C failed"); close(fd); return 1;
    }

    // CTRL1_XL: ODR_XL=104Hz (0b0100 << 4), FS=±2g (00), BW=400Hz (00) => 0x40 :contentReference[oaicite:21]{index=21}
    if (i2c_write_reg(fd, ADDR_LSM6DS33, LSM6_CTRL1_XL, 0x40) < 0) {
        die("LSM6DS33 write CTRL1_XL failed"); close(fd); return 1;
    }

    // CTRL2_G: ODR_G=104Hz (0b0100 << 4), FS=245 dps (00) => 0x40 :contentReference[oaicite:22]{index=22}
    if (i2c_write_reg(fd, ADDR_LSM6DS33, LSM6_CTRL2_G, 0x40) < 0) {
        die("LSM6DS33 write CTRL2_G failed"); close(fd); return 1;
    }

    // ---------- Identify LIS3MDL ----------
    uint8_t who_m = 0;
    if (i2c_read_regs(fd, ADDR_LIS3MDL, LIS_WHO_AM_I, &who_m, 1) < 0) {
        die("LIS3MDL WHO_AM_I read failed"); close(fd); return 1;
    }
    printf("LIS3MDL WHO_AM_I = 0x%02X (expected 0x3D)\n", who_m);

    // ---------- Configure LIS3MDL ----------
    // We'll set:
    // CTRL_REG1: Ultra-high-performance XY (OM=11), 10 Hz (DO=100), TEMP_EN=0, FAST_ODR=0 => 0b0 11 100 0 0 = 0x70 :contentReference[oaicite:23]{index=23}
    // CTRL_REG2: FS=±4 gauss => 0x00 :contentReference[oaicite:24]{index=24}
    // CTRL_REG3: Continuous-conversion mode (MD=00). Default after reset is 0x03 (power-down) per register map, so we must set 0x00. :contentReference[oaicite:25]{index=25}
    // CTRL_REG4: Ultra-high-performance Z (OMZ=11) => 0x0C (bits 3:2 = 11)
    // CTRL_REG5: BDU=1 => 0x40 (bit6)
    if (i2c_write_reg(fd, ADDR_LIS3MDL, LIS_CTRL_REG1, 0x70) < 0) { die("LIS3MDL write CTRL_REG1 failed"); close(fd); return 1; }
    if (i2c_write_reg(fd, ADDR_LIS3MDL, LIS_CTRL_REG2, 0x00) < 0) { die("LIS3MDL write CTRL_REG2 failed"); close(fd); return 1; }
    if (i2c_write_reg(fd, ADDR_LIS3MDL, LIS_CTRL_REG3, 0x00) < 0) { die("LIS3MDL write CTRL_REG3 failed"); close(fd); return 1; }
    if (i2c_write_reg(fd, ADDR_LIS3MDL, LIS_CTRL_REG4, 0x0C) < 0) { die("LIS3MDL write CTRL_REG4 failed"); close(fd); return 1; }
    if (i2c_write_reg(fd, ADDR_LIS3MDL, LIS_CTRL_REG5, 0x40) < 0) { die("LIS3MDL write CTRL_REG5 failed"); close(fd); return 1; }

    // ---------- Read loop ----------
    // Scale factors (based on chosen full-scale settings):
    // Accel ±2g sensitivity: 0.061 mg/LSB :contentReference[oaicite:26]{index=26}
    // Gyro  ±245 dps sensitivity: 8.75 mdps/LSB :contentReference[oaicite:27]{index=27}
    // Mag   ±4 gauss sensitivity: 6842 LSB/gauss :contentReference[oaicite:28]{index=28}
    const double acc_mps2_per_lsb = 0.000061 * 9.80665;   // mg -> g -> m/s^2
    const double gyr_dps_per_lsb  = 0.00875;              // mdps -> dps
    const double mag_gauss_per_lsb = 1.0 / 6842.0;        // gauss/LSB

    while (1) {
        // --- Gyro + Accel (12 bytes total if you read both separately; here we read separately for clarity) ---
        uint8_t gbuf[6], abuf[6];

        if (i2c_read_regs(fd, ADDR_LSM6DS33, LSM6_OUTX_L_G, gbuf, 6) < 0) { die("Read gyro failed"); break; }
        if (i2c_read_regs(fd, ADDR_LSM6DS33, LSM6_OUTX_L_XL, abuf, 6) < 0) { die("Read accel failed"); break; }

        int16_t gx = le16_to_i16(gbuf[0], gbuf[1]);
        int16_t gy = le16_to_i16(gbuf[2], gbuf[3]);
        int16_t gz = le16_to_i16(gbuf[4], gbuf[5]);

        int16_t ax = le16_to_i16(abuf[0], abuf[1]);
        int16_t ay = le16_to_i16(abuf[2], abuf[3]);
        int16_t az = le16_to_i16(abuf[4], abuf[5]);

        // --- Magnetometer (LIS3MDL) ---
        // For multi-byte read, set MSB of register address to enable auto-increment. :contentReference[oaicite:29]{index=29}
        uint8_t mbuf[6];
        uint8_t start = (uint8_t)(LIS_OUT_X_L | 0x80);
        if (i2c_read_regs(fd, ADDR_LIS3MDL, start, mbuf, 6) < 0) { die("Read mag failed"); break; }

        int16_t mx = le16_to_i16(mbuf[0], mbuf[1]);
        int16_t my = le16_to_i16(mbuf[2], mbuf[3]);
        int16_t mz = le16_to_i16(mbuf[4], mbuf[5]);

        // Convert to engineering units (optional)
        double ax_mps2 = ax * acc_mps2_per_lsb;
        double ay_mps2 = ay * acc_mps2_per_lsb;
        double az_mps2 = az * acc_mps2_per_lsb;

        double gx_dps = gx * gyr_dps_per_lsb;
        double gy_dps = gy * gyr_dps_per_lsb;
        double gz_dps = gz * gyr_dps_per_lsb;

        double mx_g = mx * mag_gauss_per_lsb;
        double my_g = my * mag_gauss_per_lsb;
        double mz_g = mz * mag_gauss_per_lsb;

        printf("ACC  raw[%6d %6d %6d]  m/s^2[%7.3f %7.3f %7.3f] | ",
               ax, ay, az, ax_mps2, ay_mps2, az_mps2);
        printf("GYR  raw[%6d %6d %6d]  dps  [%7.3f %7.3f %7.3f] | ",
               gx, gy, gz, gx_dps, gy_dps, gz_dps);
        printf("MAG  raw[%6d %6d %6d]  gauss[%7.4f %7.4f %7.4f]\n",
               mx, my, mz, mx_g, my_g, mz_g);

        usleep(100000); // 10 Hz print
    }

    close(fd);
    return 0;
}
