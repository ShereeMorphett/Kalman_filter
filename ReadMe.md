### This is in the process of a major rewrite


# Kalman Filter Implementation in C++

## Overview

This project implements a Kalman Filter in C++ to estimate the state of a generic vehicle moving along its longitudinal axis in an environment devoid of air and gravity. The filter processes various sensor inputs, including accelerometer, gyroscope, and GPS data, all affected by Gaussian white noise. The primary objective is to accurately estimate the vehicle's position and velocity over trajectories lasting up to 90 minutes.

## Features

- **Sensor Fusion**: Integrates accelerometer, gyroscope, and GPS data to provide a comprehensive state estimation.
- **Noise Handling**: Effectively manages Gaussian white noise in sensor measurements.
- **Performance**: Optimized for real-time processing of trajectories up to 90 minutes without timeouts or crashes.
- **Robustness**: Gracefully handles estimation errors, ensuring the system remains stable even when predictions exceed allowed thresholds.

## Sensor Specifications

- **Accelerometer**:
  - Standard Deviation (σ): 0.001 m/s²
  - Mean (υ): 0
- **Gyroscope**:
  - Standard Deviation (σ): 0.01 rad/s
  - Mean (υ): 0
- **GPS**:
  - Standard Deviation (σ): 0.1 m
  - Mean (υ): 0

## Build System
  Meson

## Installation

1. **Build the Project**:

   ```bash
   meson setup build
   meson compile -C build
   ```

3. **Run the Application**:

   ```bash
   ./imu-sensor-stream-linux -p 8080
   ```
-p will take in port. Kalman filter runs on 8080 by default
   ```bash
   ./build/bin/kalman_filter
   ```

## Usage

The application processes sensor data to estimate the vehicle's position and velocity.


## License

This project is licensed under the MIT License.
