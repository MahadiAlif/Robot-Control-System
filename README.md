# Team Roboto 2024 Robotics System

This repository contains the complete code and documentation for Team Roboto’s 2024 robotics system, featuring three specialized robots: the **Standard Robot**, **Sentry Robot**, and **Balancing Robot**. Each robot has been developed with cutting-edge technologies and refined for competitive performance, focusing on reliability, advanced control, and modularity.

## Robot Overview

### Standard Robot
Designed with a robust four-omni-wheel configuration, the Standard Robot is optimized for agility and power efficiency, featuring:
- **Chassis**: Monocoque design with high-performance 3D-printed materials for lightweight durability.
- **Suspension and Head Systems**: Advanced suspension and a high-torque head mechanism for enhanced stability and precision.
- **Shooting System**: A lightweight pitch mechanism and optimized projectile pipeline for fast, accurate firing.

### Sentry Robot
Built on a modified Standard Robot chassis, the Sentry Robot is equipped with defensive and offensive upgrades to meet competition demands:
- **Upper Protection**: Shields electronic components and improves durability.
- **Enhanced Head Design**: Holds up to 700 projectiles, with computer vision for precise targeting and a double cannon system.

### Balancing Robot
The latest Balancing Robot model includes innovative features to improve stability, maneuverability, and adaptability:
- **Movable Supports**: Leg-like supports enhance vertical movement and control.
- **Motorized Gimbal**: Redesigned for optimal balance and compactness, integrated with AK60 and AK70 CubeMars motors for high performance.
- **Control Algorithms**: Equipped with advanced algorithms (LQR and custom routines) for smooth navigation, obstacle management, and autonomous braking.

## Control System
The control system includes a modular, refactored codebase supporting enhanced mobility, aiming accuracy, and improved movement algorithms for all robots. The system is designed to facilitate rapid adjustments, including:
- **Pilot Interface**: Real-time visual feedback for impacts, assisting pilots with intuitive robot control.
- **Auto-Aiming**: YOLOv8-based detection with trajectory prediction for precise targeting.
- **Navigation**: Autonomous navigation integrating IMU, LIDAR, and visual SLAM for accurate movement and obstacle handling.

## Electronics and Communication
The electronics architecture incorporates efficient microcontroller-Jetson PC communication and a dual power supply system:
- **Pilot Interface and IMU**: Simplified pilot interface and improved IMU sensor fusion for reliable movement tracking.
- **Microcontroller-Jetson Communication**: Robust USART communication, optimized for high-speed data transfer.
- **Power System**: Dual power setup with a supercapacitor for peak energy needs, enhancing reliability during intense competition.

## Future Enhancements
Upcoming improvements focus on material optimizations, defensive features, and advanced navigation capabilities:
- **Standard Robot**: Further omni-wheel and shooting system optimization.
- **Sentry Robot**: Computer vision and cannon system upgrades.
- **Balancing Robot**: Improved motor control algorithms and structural enhancements for competitions.
