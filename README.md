# Robot Control System 🤖🎮

[![STM32 Firmware](https://img.shields.io/badge/Firmware-STM32F4-blue.svg)](https://github.com/MahadiAlif/Robot-Control-System/tree/main/firmware)
[![Jetson Nano UART](https://img.shields.io/badge/Jetson-Python--UART-orange.svg)](https://github.com/MahadiAlif/Robot-Control-System/tree/main/jetson_uart)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](https://github.com/MahadiAlif/Robot-Control-System/tree/main/jetson_uart/LICENSE)

A high-performance, real-time robotics control system featuring multi-agent support for **Standard**, **Sentry**, and **Balancing** robots. This architecture coordinates an onboard **STM32F407IGH6** controller running **FreeRTOS** C firmware with an **NVIDIA Jetson Nano** companion node handling AI computer vision target-tracking via an optimized, high-frequency UART telemetry protocol.

---

## 🛠️ System Architecture

`mermaid
graph TD
    subgraph Jetson Nano (Companion Node)
        AI[AI Vision Target Detection] -->|Float Array| UART_TX[UART Controller: rx_control_final.py]
    end

    subgraph STM32F407IGH6 Controller (FreeRTOS)
        UART_RX[AI_receive USART Task] -->|Parsed Telemetry| INS[INS_task: BMI088 + IST8310]
        INS -->|Fused Orientation| Control[Gimbal & Chassis Controllers]
        DBUS[DBUS Remote Control Task] -->|User Input| Control
        Control -->|CAN Telemetry| Motors[RoboMaster CAN Motors: C610, C620, GM6020]
        Control -->|Firing Commands| Shooter[Shooting Task & Bullet Feed]
        Referee[Referee Usart Decoder] -->|Game State| Control
    end

    UART_TX <==>|115,200 Baud Serial| UART_RX
    Motors <==>|CAN Bus 1 & 2| Control
`

---

## 📂 Repository Structure

The project has been restructured for modularity and easy compilation:

`
Robot-Control-System/
├── .gitignore                      # Workspace ignore filters (excludes Keil object files)
├── README.md                       # Comprehensive system documentation
├── firmware/                       # STM32 CubeMX & Keil MDK-ARM project
│   ├── Src/ & Inc/                 # HAL generated source and header core
│   ├── Drivers/ & Middlewares/     # HAL Drivers & FreeRTOS kernel libraries
│   ├── MDK-ARM/                    # Keil MDK-ARM project targets (can.uvprojx)
│   ├── application/                # Core robot application tasks (Chassis, Gimbal, INS, Shoot)
│   ├── bsp/                        # Board Support Package drivers (CAN, SPI, USART, RC, Delay)
│   └── components/                 # Sensor drivers, controllers, PID, and AHRS fusion filters
├── jetson_uart/                    # Companion node scripts on Jetson Nano
│   ├── requirements.txt            # Python dependencies (pyserial, keyboard)
│   ├── rx_control_final.py         # Main computer vision float decoding & telemetry controller
│   └── uart_communication.py       # Basic loopback check and diagnostic testing
├── docs/                           # Hardware specification libraries
│   ├── datasheets/                 # STM32 manual & Jetson board spec sheet
│   ├── manuals/                    # RoboMaster Board Type C user manual
│   └── schematics/                 # Schematic PDFs (mainboard, gyroscope)
└── media/                          # Screenshots and layout graphics
`

---

## 🛰️ UART Communication Protocol

The Jetson and STM32 communicate over an optimized serial interface at **115,200 baud**:

* **Synchronization Frame Start**: ASCII % (value 37).
* **Telemetry Data Packet**: Space-separated decimal string representing target pitch, yaw, and range coordinates, followed by validation checksum.
* **Payload Conversion**: The Jetson parses and converts the byte strings into raw floats in real-time under \text{ms}$ using x_control_final.py.

---

## ⚡ FreeRTOS Real-Time Task Allocation

The STM32 firmware allocates scheduling priority to crucial tasks under FreeRTOS to prevent race conditions:

| Task Name | Priority | Core File | Responsibility |
| :--- | :---: | :--- | :--- |
| **INS Task** | Real-Time | INS_task.c | Fuses 6-axis BMI088 IMU + IST8310 Magnetometer via AHRS quaternion filters at \text{kHz}$. |
| **Chassis Task** | High | chassis_task.c | Wheel speed kinematic calculations (Standard, Sentry omni-wheels, or Balancing leg PID). |
| **Gimbal Task** | High | gimbal_task.c | Stabilizes yaw and pitch axes using IMU absolute orientation feeds. |
| **Shoot Task** | Medium | shooting_task.c | Controls bullet feeding frequency, projectile friction wheels, and safety checks. |
| **AI Receive** | Medium | AI_receive.c | Receives and validates target tracking data from the Jetson serial stream. |
| **Referee Task** | Low | eferee.c | Decodes server packets (power constraints, bullet limits, robot health). |

---

## 🔧 Installation & Build Instructions

### 1. Jetson Nano Software Setup
Clone the repository and install serial dependencies:
`ash
cd jetson_uart
pip3 install -r requirements.txt
`
To run the telemetry controller:
`ash
sudo python3 rx_control_final.py
`

### 2. STM32 Firmware Compilation
1. Open the Keil workspace located at irmware/MDK-ARM/can.uvprojx.
2. Click **Build** (F7) to compile. The .gitignore prevents tracking compiled object files, keeping your checkouts completely clean.
3. Use a ST-Link / J-Link to flash the binary to the **RoboMaster Development Board Type C (STM32F407IGH6)**.
