# AquaPing

**AquaPing** is a compact underwater altimetry and acoustic telemetry system that transmits real-time depth measurements from a Ping1D altimeter via Delphis acoustic modems to a surface receiver (e.g., a boat). It is designed for underwater robotics, seabed profiling, or oceanographic experiments.

---

## 🚀 Features

- 📡 Real-time acoustic transmission using Delphis modems
- 📏 Depth measurements from Ping1D (Blue Robotics)
- 🔁 Automatic retry and error handling
- ⚙️ Compact 3D-printed mounts
- 🛥️ Receiver software for boats or surface stations

---

## 🛠️ Components

### Hardware

- **Raspberry Pi Zero / Pi 3 / 4**
- **Ping1D Altimeter** (by Blue Robotics)
- **Delphis Acoustic Modem (2x)** – One for transmission, one for reception
- **12V battery supply / power regulation**
- **3D-printed housing** (STL files included)

### Software

- Python 3
- `brping` (Blue Robotics Ping1D library)
- PySerial
- RPi.GPIO

---

## 📁 Repository Structure

| Folder        | Description                              |
|---------------|------------------------------------------|
| `/transmitter`| Python script for the underwater unit    |
| `/receiver`   | Script for receiving data at the surface |
| `/3d_models`  | 3D-printed enclosures and mounts (STL)   |
| `/docs`       | Photos, diagrams, or setup instructions  |

---

## 📸 System Overview

![System Diagram](docs/system_diagram.png)

---

## 🔧 Installation

### On the Transmitter (Raspberry Pi):

```bash
sudo apt update
sudo apt install python3-pip python3-serial
pip3 install brping
