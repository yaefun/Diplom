# ESP32-CAM Object Detection System

A small embedded system based on the ESP32-CAM AI Thinker that detects nearby objects using an ultrasonic sensor and automatically captures images depending on how long the object remains in the detection zone.

## Overview

The system combines an ESP32-CAM, an OV2640 camera, an HC-SR04 ultrasonic sensor and a microSD card.

The ultrasonic sensor continuously measures the distance to an object.

The system uses the following logic:

- Object farther than 50 cm → system waits.
- Object appears closer than 50 cm → object tracking starts.
- Object leaves before 5 seconds → a single photo is saved.
- Object remains for 5 seconds → 25 JPEG frames are captured.
- After an event, the system waits until the object leaves before detecting another event.

This prevents repeated captures while the same object remains in front of the sensor.

---

## Features

- Automatic object detection by distance
- ESP32-CAM AI Thinker
- OV2640 camera
- HC-SR04 ultrasonic sensor
- microSD card storage
- Automatic photo capture
- 5-second object tracking
- 25-frame capture sequence
- Distance logging
- Automatic file numbering
- Protection against repeated detection of the same object
- Built-in camera flash LED

---

## Hardware

- ESP32-CAM AI Thinker
- OV2640 camera
- HC-SR04 ultrasonic distance sensor
- microSD card
- 5V power source

---

## Pin Configuration

### HC-SR04

| HC-SR04 | ESP32-CAM |
|---------|-----------|
| VCC | 5V |
| GND | GND |
| TRIG | GPIO12 |
| ECHO | GPIO13 |

### Camera

The camera uses the standard AI Thinker ESP32-CAM pin configuration.

### Flash LED

The built-in flash LED is connected to:

```text
GPIO4
