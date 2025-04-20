# Robohand - Raspberry Pi Pico Robotic Hand Controller

## Overview

Robohand is a lightweight, extensible, multithreaded robotic hand control system built for the Raspberry Pi Pico using the Pico SDK and MicroROS library. The system provides a comprehensive interface for controlling servo motors and reading from various sensors, with support for multiple communication interfaces.

## Directory Structure
./Include

├── Robohand_callbacks.h

├── Robohand_common.h

├── Robohand_dma.h

├── Robohand.h

├── Robohand_i2c.h

├── Robohand_init.h

├── Robohand_interrupts.h

├── Robohand_reader.h

├── Robohand_rgb.h

├── Robohand_servos.h

├── Robohand_struct.h

├── Robohand_timing.h

└── Robohand_uros.h

./Src

├── Core

│   ├── Robohand_callbacks.c

│   ├── Robohand_dma.c

│   ├── Robohand_i2c.c

│   ├── Robohand_init.c

│   ├── Robohand_interrupts.c

│   ├── Robohand_reader.c

│   ├── Robohand_rgb.c

│   ├── Robohand_servos.c

│   ├── Robohand_struct.c

│   └── Robohand_timing.c

├── Robohand.c

├── Robohand_uros.c

└── Robohand_usb.c

## Hardware Configuration

The system is designed to work with the following hardware:

- 5 MG996R Servos controlled via PWM on GPIO 11-15
- ADS1115 ADC connected to I2C0 (GPIO 0 & 1)
- BME/P280 pressure/temperature/humidity sensor on I2C0 (optional interrupt on GPIO X)
- MPU6050 accelerometer/gyroscope on I2C0 (optional interrupt on GPIO X)
- QMC5883L magnetometer on I2C0 (optional interrupt on GPIO X)
- Optional Common Cathode RGB LED on GPIO 16-18

## Software Architecture

The code is organized into several modules:
- **Robohand_callbacks.c/h**: Utilizes a series of timer callbacks to mark devices as ready to be read from. Slowest backend.
- **Robohand_dma.c/h**: Utilizes Direct Memory Access to offload device interaction to DMA controllers, the core can retrieve memory from a well known location when DMA signals that the buffer is ready.
- **Robohand_init.c/h**: Initialize device connections, and prepare system for operation.
- **Robohand_interrupts.c/h**: Utilizes signals from a physical interrupt pin to mark devices as ready to be read from. Middle of the line performance suspected.
- **Robohand_i2c.c/h**: I2C initialization, error checking, and utility functions. Uses Pico SDK
- **Robohand.c/h**: Core system hardware implementation, providing the interface to physical hardware via Pico SDK.
- **Robohand_usb.c**: Provides a terminal interface for monitoring and controlling the hand over USB.
- **Robohand_uros.c/h**: MicroROS client implementation for ROS integration.

The system runs on dual cores:
- **Core 0**: Handles communication with host systems (USB or MicroROS)
- **Core 1**: Manages hardware I/O, sensor polling, and servo control

## Features

- **Multithreaded Design**: Utilizes both cores of the RP2040 for parallel processing
- **Flexible Communication**: USB terminal interface or ROS integration via MicroROS
- **Comprehensive Sensor Support**: Read from multiple sensors over I2C
- **Smooth Motion Control**: Implements trapezoidal motion profiles for servo control
- **Safety Features**: Watchdog timer, emergency stop functionality, mutex protection
- **Multiple Backend Options**: Choose between interrupt-based, DMA, or callback-based communication

## Configuration Options

The system can be configured by modifying the defines in `Robohand.h`:

```c
#define HAS_ADS1115 true    // Enable ADS1115 ADC
#define HAS_BME280 true     // Enable BME280 sensor
#define HAS_QMC5883L true   // Enable QMC5883L magnetometer
#define HAS_MPU6050 true    // Enable MPU6050 IMU
#define HAS_PI_ADC false    // Enable Pico's internal ADC
#define HAS_RGB true        // Enable RGB LED indicator
#define HAS_SERVOS true     // Enable servo control

#define USE_INTERRUPTS false // Use interrupt-based communication
#define USE_DMA false        // Use DMA-based communication
#define USE_CALLBACKS true   // Use callback-based communication
```

## Building the Project

### Prerequisites

- Raspberry Pi Pico SDK (v1.4.0 or later)
- CMake (v3.13 or later)
- GCC ARM toolchain
- MicroROS library (for ROS integration)

### Setup

1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/robohand.git
   cd robohand
   ```

2. Create a build directory:
   ```bash
   mkdir build
   cd build
   ```

3. Configure and build:
   ```bash
   cmake ..
   make
   ```

This will build two targets:
- `Robohand_usb.uf2` - USB terminal interface
- `Robohand_uros.uf2` - MicroROS interface

### Flashing

1. Hold the BOOTSEL button on the Pico while connecting it to the computer
2. Copy the desired .uf2 file to the mounted Pico drive:
   ```bash
   cp Robohand_usb.uf2 /media/username/RPI-RP2/
   ```

## Usage

### USB Terminal Interface

Connect to the Pico's USB serial port:
```bash
screen /dev/ttyACM0 115200
```

Available commands:
- `status` - Display system status
- `sensors` - Show current sensor readings
- `servo <index> <position> <duration>` - Move a servo
- `rgb <r> <g> <b>` - Set RGB LED color
- `blink <on|off> <interval>` - Control LED blinking
- `help` - Show available commands

### MicroROS Interface

When using the MicroROS build, the system will automatically connect to a MicroROS agent and advertise the following topics:

- `/robohand/servos` - Control servos
- `/robohand/sensors` - Publish sensor data
- `/robohand/status` - Publish system status

## Performance Considerations

- **Servo Control**: For smooth motion, keep update intervals below 20ms
- **Sensor Polling**: Using interrupts or DMA reduces CPU load but increases complexity
- **Memory Usage**: The system uses static allocation for critical structures to avoid heap fragmentation

## Troubleshooting

- **Terminal Backspacing Issues**: Make sure to use the latest build with CRLF fixes
- **Servo Jittering**: Check power supply stability and reduce update frequency
- **I2C Communication Failures**: Verify wiring and pull-up resistors
- **System Crashes**: Enable watchdog timer and check for mutex deadlocks

## License

This project is licensed under the Apache 2.0 License - see the LICENSE file for details.

## Author

Robert Fudge <rnfudge@mun.ca>

## Acknowledgments

- Raspberry Pi Foundation for the Pico SDK
- MicroROS team for embedded ROS support