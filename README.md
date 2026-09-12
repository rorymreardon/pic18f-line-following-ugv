# Autonomous Line-Following UGV Firmware (PIC18F46K22)

Embedded C firmware for a 4-wheeled autonomous ground vehicle that
follows a line using 3 IR sensors and stops for obstacles using an
HC-SR04 ultrasonic sensor. Built as part of a 4-person team project
(Group 3: UGV); this repo covers the firmware I wrote.

## Hardware

- **MCU:** PIC18F46K22, running at 16 MHz (internal oscillator)
- **Chassis:** custom 3D-modeled and 3D-printed body (Blender, CR-PETG)
- **Sensing:** HC-SR04 ultrasonic sensor (obstacle detection), 3x
  TCRT5000 IR sensors (left/mid/right line detection)
- **Actuation:** 4x TT motors driven through a TB6612FNG H-bridge, PWM
  speed control via CCP1/CCP2

## How It Works

**Line following** reads the 3 IR sensors as a 3-bit pattern
(`L M R`) and maps each of the 8 possible patterns to a driving
behavior — centered, slight/hard correction, intersection handling, or
a "line lost" recovery mode that searches in the last known direction
based on `last_error`.

**Obstacle avoidance** triggers the HC-SR04, measures the echo pulse
width using Timer1, converts it to inches, and stops the vehicle if an
object is within `STOP_DISTANCE_IN` (5 inches).

## Bug Fixed From the Original Version

The original `Timer1_Init()` set `T1CON = 0b00110000`, which selects a
**1:8** prescaler — but the surrounding comments (and the math in
`get_distance_inches()`) assumed a **1:4** prescaler (1 microsecond
per timer tick). With the 1:8 setting, each tick was actually ~2
microseconds, so the computed distance was roughly double the true
distance.

This version sets `T1CON = 0b00100000` (1:4 prescale), so 1 Timer1
tick genuinely equals 1 microsecond at 16 MHz, matching what
`get_distance_inches()` assumes when it divides by 148.

## Tools

- C (MPLAB X / XC8 compiler)
- PIC18F46K22 (8-bit PIC microcontroller)
- Blender (chassis modeling), 3D printing (CR-PETG)
