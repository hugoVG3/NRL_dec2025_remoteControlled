# NRL_dec2025_remoteControlled

A PlatformIO-based ESP32 project for remotely controlling a robot using a PS5 (DualSense) controller for the **2025 National Robotics League (NRL)** competition.

This repository contains the firmware responsible for receiving controller input, interpreting it, and transmitting robot-control signals in real time. The project is designed for reliability, low latency, and clean integration with robotics hardware.

---

## 🚀 Project Overview

This project implements a remote-control interface for a competition robot, enabling operators to control movement and mechanisms using a standard PS5 controller. The firmware runs on an ESP32 using the PlatformIO environment, chosen for its portability, build consistency, and embedded-development tooling.

The system’s responsibilities include:

- Managing Bluetooth communication with the controller  
- Reading and processing DualSense input states  
- Translating controller inputs into robot-control commands  
- Maintaining responsiveness required for NRL competition performance  

The goal is a clean, maintainable, and competition-ready control layer that can be extended to any robot architecture.

---

## 🔧 PS5 Controller Library

This project uses the **ESP_ps5_control** library:

👉 https://github.com/hugoVG3/ESP_ps5_control.git

This library provides a minimal, single-header interface for communicating with a PS5 DualSense controller over Bluetooth Classic. It supports:

- Digital button input  
- Analog sticks  
- Triggers  
- Basic output features (LED and rumble)  
- Battery/charging status reporting  

Its simplicity and low overhead make it ideal for robotics applications, where predictable control response is crucial.

---

## 📁 Repository Structure

NRL_dec2025_remoteControlled/

├── include/ # Project headers and PS5 controller interface

├── lib/ # Optional libraries

├── src/ # Core logic for processing controller input and robot control

├── platformio.ini # PlatformIO configuration and build settings

└── .gitignore

This structure follows the standard PlatformIO layout for clarity and scalability.

---

## 🎯 Purpose in the NRL 2025 Competition

The code in this repository forms the control backbone of an NRL 2025 competition robot.  
It enables:

- Smooth operator control via a familiar and ergonomic interface  
- Strong separation between high-level control logic and low-level robot hardware  
- Easy future expansion for autonomous features, additional actuators, and fail-safe mechanisms  

The focus is on robustness, readability, and competition practicality.

---

## 📜 License

The PS5 controller library (`ESP_ps5_control`) is licensed under MIT, so is this one :)

---

## 📌 Links

- ESP_ps5_control library: https://github.com/hugoVG3/ESP_ps5_control.git  
- This repository: https://github.com/hugoVG3/NRL_dec2025_remoteControlled.git  

---

If you have suggestions or would like to adapt this structure for other robotics projects, feel free to contribute.
