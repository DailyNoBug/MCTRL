# MCtrl - Quadcopter Smart Controller
## Overview
A high-performance remote control system for quadcopters, integrating flight control, FPV video transmission, parameter tuning, and edge computing. Combines an STM32G431 microcontroller for real-time control and an RK3588S SoC running Android 12 for advanced processing, achieving sub-50ms control latency and 5km operational range.

## Key Features
📶 Dual-Band Communication

2.4GHz control link (VG2392S240X0M2) + 5.8GHz HD video stream (BL-8821EU2 WiFi6)

🎮 Precision Control

Dual analog joysticks with auto-calibration
Fail-safe protocol for signal loss

📺 7" Touch Interface

Real-time telemetry display
On-screen PID tuning and flight mode switching

🖥 Edge Computing (not implemented)

Object detection via TensorFlow Lite
OpenCV-based video processing pipeline

## Hardware Architecture

| Component	             | Specification |
|------------------------| --- |
| Main Controller	       | STM32G431 (Cortex-M4) |
| Soc                    | RK3588S (8-core ARM) |
| Wireless Module	       | VG2392S240X0M2 (2.4GHz, 30dBm) |
| video wireless Module	 | BL-8821EU2 (802.11ax, 30dBm) |
| Display	               | 7" IPS (1280×800, 10-point touch) |
| Battery	               | 2S 5000mAh Li-ion |

## Software Stack
### Firmware
FreeRTOS-based control system
Custom binary protocol for low-latency communication

### Android App
Video decoding pipeline (H.264/H.265)
Flight data visualization dashboard

## Build & Usage
Flash STM32 firmware via ST-Link

Install Android APK on RK3588S

Pair controller with MFly quadcopter in another repo