# BMP280 Temperature Monitor — QNX RTOS / Raspberry Pi 4

Reads ambient temperature from a BMP280/BME280 sensor over the I2C bus on a Raspberry Pi 4 running QNX Neutrino RTOS. Uses a POSIX real-time timer (`timer_create`) to schedule periodic sensor reads every 10 seconds.

## Hardware Setup

| BMP280 Pin | RPi 4 Pin |
|------------|-----------|
| VCC | 3.3V |
| GND | GND |
| SDA | GPIO 2 |
| SCL | GPIO 3 |

## How It Works

- Auto-detects sensor at I2C address `0x76` or `0x77`
- Reads factory calibration registers (T1, T2, T3) on startup
- Configures BMP280 in normal mode with 2x temperature oversampling
- Fires a `SIGRTMIN` real-time signal every 10 seconds via `timer_settime`
- Compensates raw ADC value using the BMP280 datasheet formula

## Build & Run (QNX)

```bash
qcc -o bmp280_monitor rtos_project_i2c_new.c -l i2c
./bmp280_monitor
```

## Sample Output

```
BMP280 Temperature Reader - QNX/RPi4
====================================
Sensor detected at address 0x76
Calibration data: T1=27504, T2=26435, T3=-1000
BMP280 configured
Timer started. Reading temperature every 10 seconds...

[1748823600] Temperature: 28.45 °C
[1748823610] Temperature: 28.47 °C
```

## Tech Stack

- **Platform:** Raspberry Pi 4
- **OS:** QNX Neutrino RTOS
- **Protocol:** I2C (`/dev/i2c1`, QNX `devctlv` API)
- **Sensor:** Bosch BMP280 / BME280
- **Timing:** POSIX real-time timers (`CLOCK_REALTIME`, `SIGRTMIN`)
- **Language:** C
