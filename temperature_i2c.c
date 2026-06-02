
/*
 * Problem statement:
 * Read ambient temperature from a BMP280 sensor over the I2C bus on a Raspberry Pi 4
 * Display the temperature reading on the console every 10 seconds.
 * Use a timer to schedule periodic readings.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <hw/i2c.h>
#include <sys/iomsg.h>
#include <time.h>
#include <signal.h>
#include <string.h>
#include <errno.h>

// BMP280 I2C addresses (common values)
#define BMP280_I2C_ADDR_PRIMARY   0x76
#define BMP280_I2C_ADDR_SECONDARY 0x77

// BMP280 register addresses
#define BMP280_REG_CHIP_ID        0xD0
#define BMP280_REG_RESET          0xE0
#define BMP280_REG_CTRL_MEAS      0xF4
#define BMP280_REG_CONFIG         0xF5
#define BMP280_REG_TEMP_MSB       0xFA
#define BMP280_REG_TEMP_LSB       0xFB
#define BMP280_REG_TEMP_XLSB      0xFC

// BMP280 calibration register base
#define BMP280_REG_CALIB_BASE     0x88

// Expected chip IDs
#define BMP280_CHIP_ID            0x58
#define BME280_CHIP_ID            0x60

// I2C device path (adjust for your QNX configuration)
#define I2C_DEVICE                "/dev/i2c1"

// Sampling interval in seconds
#define SAMPLE_INTERVAL_SEC       10

// BMP280 calibration data
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
} bmp280_calib_t;

// Global variables
static int i2c_fd = -1;
static uint8_t g_i2c_addr = BMP280_I2C_ADDR_PRIMARY;
static bmp280_calib_t calib;
static volatile sig_atomic_t keep_running = 1;

// Function prototypes
static int i2c_init(const char *device, uint8_t addr);
static int i2c_write_byte(int fd, uint8_t reg, uint8_t value);
static int i2c_read_bytes(int fd, uint8_t reg, uint8_t *buf, size_t len);
static int bmp280_detect(int fd);
static int bmp280_read_calibration(int fd, bmp280_calib_t *cal);
static int bmp280_configure(int fd);
static int bmp280_read_raw_temp(int fd, int32_t *adc_T);
static double bmp280_compensate_temp(int32_t adc_T, const bmp280_calib_t *cal);
static void signal_handler(int sig);
static void timer_handler(int sig, siginfo_t *si, void *uc);

int main(void) {
    struct sigaction sa;
    struct sigevent sev;
    timer_t timerid;
    struct itimerspec its;

    printf("BMP280 Temperature Reader - QNX/RPi4\n");
    printf("====================================\n\n");

    // Set up signal handler for SIGINT (Ctrl-C)
    signal(SIGINT, signal_handler);

    // Try both I2C addresses
    printf("Scanning for BMP280/BME280...\n");

    // Try primary address first
    printf("Trying address 0x%02X...\n", BMP280_I2C_ADDR_PRIMARY);
    i2c_fd = i2c_init(I2C_DEVICE, BMP280_I2C_ADDR_PRIMARY);
    if (i2c_fd >= 0) {
        g_i2c_addr = BMP280_I2C_ADDR_PRIMARY;
        if (bmp280_detect(i2c_fd) == 0) {
            printf("Sensor detected at address 0x%02X\n\n", BMP280_I2C_ADDR_PRIMARY);
            goto sensor_found;
        }
        close(i2c_fd);
        i2c_fd = -1;
    }

    // Try secondary address
    printf("Trying address 0x%02X...\n", BMP280_I2C_ADDR_SECONDARY);
    i2c_fd = i2c_init(I2C_DEVICE, BMP280_I2C_ADDR_SECONDARY);
    if (i2c_fd >= 0) {
        g_i2c_addr = BMP280_I2C_ADDR_SECONDARY;
        if (bmp280_detect(i2c_fd) == 0) {
            printf("Sensor detected at address 0x%02X\n\n", BMP280_I2C_ADDR_SECONDARY);
            goto sensor_found;
        }
        close(i2c_fd);
        i2c_fd = -1;
    }

    // Not found at either address
    fprintf(stderr, "\nERROR: BMP280/BME280 not detected at 0x%02X or 0x%02X\n",
            BMP280_I2C_ADDR_PRIMARY, BMP280_I2C_ADDR_SECONDARY);
    fprintf(stderr, "Please check:\n");
    fprintf(stderr, "  1. Sensor power (VCC to 3.3V, GND to GND)\n");
    fprintf(stderr, "  2. I2C connections (SDA to GPIO2, SCL to GPIO3)\n");
    fprintf(stderr, "  3. CSB pin connected to 3.3V (for I2C mode)\n");
    fprintf(stderr, "  4. SDO pin to GND (addr 0x76) or 3.3V (addr 0x77)\n");
    fprintf(stderr, "  5. I2C driver loaded: slay i2c-bcm2711; i2c-bcm2711\n");
    return 1;

sensor_found:
    // Read calibration data
    if (bmp280_read_calibration(i2c_fd, &calib) < 0) {
        fprintf(stderr, "Failed to read calibration data\n");
        close(i2c_fd);
        return 1;
    }
    printf("Calibration data: T1=%u, T2=%d, T3=%d\n",
           calib.dig_T1, calib.dig_T2, calib.dig_T3);

    // Configure BMP280 for normal mode
    if (bmp280_configure(i2c_fd) < 0) {
        fprintf(stderr, "Failed to configure BMP280\n");
        close(i2c_fd);
        return 1;
    }
    printf("BMP280 configured\n\n");

    // Set up timer signal handler
    sa.sa_flags = SA_SIGINFO;
    sa.sa_sigaction = timer_handler;
    sigemptyset(&sa.sa_mask);
    if (sigaction(SIGRTMIN, &sa, NULL) == -1) {
        perror("sigaction");
        close(i2c_fd);
        return 1;
    }

    // Create timer
    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_signo = SIGRTMIN;
    sev.sigev_value.sival_ptr = &timerid;
    if (timer_create(CLOCK_REALTIME, &sev, &timerid) == -1) {
        perror("timer_create");
        close(i2c_fd);
        return 1;
    }

    // Start timer (periodic every SAMPLE_INTERVAL_SEC seconds)
    its.it_value.tv_sec = SAMPLE_INTERVAL_SEC;
    its.it_value.tv_nsec = 0;
    its.it_interval.tv_sec = SAMPLE_INTERVAL_SEC;
    its.it_interval.tv_nsec = 0;

    if (timer_settime(timerid, 0, &its, NULL) == -1) {
        perror("timer_settime");
        timer_delete(timerid);
        close(i2c_fd);
        return 1;
    }

    printf("Timer started. Reading temperature every %d seconds...\n", SAMPLE_INTERVAL_SEC);
    printf("Press Ctrl-C to exit.\n\n");

    // Initial read
    int32_t adc_T;
    if (bmp280_read_raw_temp(i2c_fd, &adc_T) == 0) {
        double temp = bmp280_compensate_temp(adc_T, &calib);
        printf("[%ld] Temperature: %.2f °C\n", time(NULL), temp);
    }

    // Keep running until SIGINT
    while (keep_running) {
        pause();  // Wait for signals
    }

    // Cleanup
    printf("\nShutting down...\n");
    timer_delete(timerid);
    close(i2c_fd);
    printf("Done.\n");

    return 0;
}

// Initialize I2C device
static int i2c_init(const char *device, uint8_t addr) {
    int fd = open(device, O_RDWR);
    if (fd < 0) {
        perror("open I2C device");
        return -1;
    }
    printf("  Opened %s successfully\n", device);
    return fd;
}


// Write a byte to a register
static int i2c_write_byte(int fd, uint8_t reg, uint8_t value) {
    i2c_send_t hdr;
    iov_t siov[2];
    uint8_t buf[2] = {reg, value};

    hdr.slave.addr = g_i2c_addr;
    hdr.slave.fmt = I2C_ADDRFMT_7BIT;
    hdr.len = 2;
    hdr.stop = 1;

    // Set up scatter-gather I/O vectors
    SETIOV(&siov[0], &hdr, sizeof(hdr));
    SETIOV(&siov[1], buf, 2);

    if (devctlv(fd, DCMD_I2C_SEND, 2, 0, siov, NULL, NULL) != EOK) {
        fprintf(stderr, "  i2c_write_byte failed: %s\n", strerror(errno));
        return -1;
    }

    return 0;
}

// Read bytes from a register
static int i2c_read_bytes(int fd, uint8_t reg, uint8_t *buf, size_t len) {
    i2c_sendrecv_t hdr;
    iov_t siov[2], riov[2];

    hdr.slave.addr = g_i2c_addr;
    hdr.slave.fmt = I2C_ADDRFMT_7BIT;
    hdr.send_len = 1;
    hdr.recv_len = len;
    hdr.stop = 1;

    // Set up send I/O vectors (header + register address)
    SETIOV(&siov[0], &hdr, sizeof(hdr));
    SETIOV(&siov[1], &reg, 1);

    // Set up receive I/O vectors (header + data buffer)
    SETIOV(&riov[0], &hdr, sizeof(hdr));
    SETIOV(&riov[1], buf, len);

    if (devctlv(fd, DCMD_I2C_SENDRECV, 2, 2, siov, riov, NULL) != EOK) {
        fprintf(stderr, "  i2c_read_bytes failed: %s\n", strerror(errno));
        return -1;
    }

    return 0;
}



// Detect BMP280/BME280 by reading chip ID
static int bmp280_detect(int fd) {
    uint8_t chip_id = 0;

    printf("  Reading chip ID from register 0x%02X...\n", BMP280_REG_CHIP_ID);

    if (i2c_read_bytes(fd, BMP280_REG_CHIP_ID, &chip_id, 1) < 0) {
        fprintf(stderr, "  Failed to read chip ID register\n");
        return -1;
    }

    printf("  Chip ID: 0x%02X ", chip_id);

    if (chip_id == BMP280_CHIP_ID) {
        printf("(BMP280)\n");
        return 0;
    } else if (chip_id == BME280_CHIP_ID) {
        printf("(BME280)\n");
        return 0;
    } else if (chip_id == 0x00 || chip_id == 0xFF) {
        printf("(no response - check wiring)\n");
        return -1;
    } else {
        printf("(unknown sensor)\n");
        return -1;
    }
}

// Read calibration data
static int bmp280_read_calibration(int fd, bmp280_calib_t *cal) {
    uint8_t calib_data[6];

    if (i2c_read_bytes(fd, BMP280_REG_CALIB_BASE, calib_data, 6) < 0) {
        fprintf(stderr, "Failed to read calibration registers\n");
        return -1;
    }

    // Parse calibration (little-endian)
    cal->dig_T1 = (uint16_t)(calib_data[1] << 8 | calib_data[0]);
    cal->dig_T2 = (int16_t)(calib_data[3] << 8 | calib_data[2]);
    cal->dig_T3 = (int16_t)(calib_data[5] << 8 | calib_data[4]);

    return 0;
}

// Configure BMP280 for temperature measurement
static int bmp280_configure(int fd) {
    // Set oversampling: temp x2, mode normal (0x4F = 01001111)
    // osrs_t[7:5] = 010 (x2), osrs_p[4:2] = 001 (x1), mode[1:0] = 11 (normal)
    if (i2c_write_byte(fd, BMP280_REG_CTRL_MEAS, 0x4F) < 0) {
        fprintf(stderr, "Failed to write CTRL_MEAS register\n");
        return -1;
    }

    // Set config: standby 0.5ms, filter off
    if (i2c_write_byte(fd, BMP280_REG_CONFIG, 0x00) < 0) {
        fprintf(stderr, "Failed to write CONFIG register\n");
        return -1;
    }

    // Small delay for sensor to stabilize
    usleep(100000); // 100ms

    return 0;
}

// Read raw temperature ADC value
static int bmp280_read_raw_temp(int fd, int32_t *adc_T) {
    uint8_t data[3];

    if (i2c_read_bytes(fd, BMP280_REG_TEMP_MSB, data, 3) < 0) {
        fprintf(stderr, "Failed to read temperature registers\n");
        return -1;
    }

    // Combine to 20-bit value (MSB, LSB, XLSB[7:4])
    *adc_T = (int32_t)((data[0] << 12) | (data[1] << 4) | (data[2] >> 4));

    return 0;
}

// Compensate temperature using calibration data (from BMP280 datasheet)
static double bmp280_compensate_temp(int32_t adc_T, const bmp280_calib_t *cal) {
    double var1, var2, T;

    var1 = (((double)adc_T) / 16384.0 - ((double)cal->dig_T1) / 1024.0) * ((double)cal->dig_T2);
    var2 = ((((double)adc_T) / 131072.0 - ((double)cal->dig_T1) / 8192.0) *
            (((double)adc_T) / 131072.0 - ((double)cal->dig_T1) / 8192.0)) * ((double)cal->dig_T3);
    T = (var1 + var2) / 5120.0;

    return T;
}

// Signal handler for SIGINT
static void signal_handler(int sig) {
    (void)sig;
    keep_running = 0;
}

// Timer handler - called every SAMPLE_INTERVAL_SEC seconds
static void timer_handler(int sig, siginfo_t *si, void *uc) {
    (void)sig;
    (void)si;
    (void)uc;

    int32_t adc_T;
    if (bmp280_read_raw_temp(i2c_fd, &adc_T) == 0) {
        double temp = bmp280_compensate_temp(adc_T, &calib);
        printf("[%ld] Temperature: %.2f °C\n", time(NULL), temp);
    } else {
        fprintf(stderr, "[%ld] Failed to read temperature\n", time(NULL));
    }
}

