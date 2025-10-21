# MYBOT MCU Firmware

This repository contains the firmware source code for the **MCU components** of the **MYBOT project** — a self-developed autonomous robot that serves as the **Bachelor’s Thesis** of **Nguyen Thanh Tai**.  
> Note: This project is **not fully open-source**. Only the MCU firmware part is available here.

For more information and a demonstration of the MYBOT project, check out the video below:

👉 https://www.youtube.com/watch?v=H-uJPVZc3JQ

## Overview

The repository includes firmware for **two microcontrollers (MCUs)** used in the robot system:

<img width="653" height="401" alt="System_Architecture drawio" src="https://github.com/user-attachments/assets/9b9e8580-e57d-4c21-a74c-fb10fce94eff" />

### 1. ESP32 (Micro-ROS)
- Acts as a **slave device** responsible for reading peripheral data (battery, IMU, LEDs, etc.).
- Communicates **directly with the robot’s main computer** (running ROS 2) via **Serial (Micro-ROS transport)**.

### 2. Arduino Nano (Traditional Firmware)
- Functions as a **slave motor controller**.
- Receives **motion control commands** from the robot’s computer **UART communication**.
- Sends **encoder feedback** back to the computer via **UART communication**.
