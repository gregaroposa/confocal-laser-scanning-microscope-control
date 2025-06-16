# Confocal Laser Scanning Microscope Control

This repository contains the code used to operate a small confocal laser scanning microscope. The project is split between a Python application that provides a GUI and data processing routines, and an Arduino sketch that directly controls the hardware.

## Python application

The `python/` directory implements the desktop side of the controller:

- **`gui.py`** – Tkinter/ttkbootstrap based interface with tabs for galvo control, manual Z movement, Z‑scans, autofocus, raster scans and layered 3‑D scanning.
- **`comms.py`** – `SerialManager` class simplifying communication with the Arduino over a serial port.
- **`packets.py`** – helpers to encode commands (galvo moves, stepper operations, scan requests) into the binary packets expected by the firmware.
- **`scanners/`** – classes implementing different scan strategies (single galvo point, raster scan, Z‑scan, autofocus and 3‑D layered scan).
- **`processing.py`** – utilities for cleaning and smoothing photodiode signals.
- **`main.py`** – entry point that launches the GUI (`python -m python.main`).

The application depends on `pyserial`, `numpy`, `scipy`, `matplotlib` and `ttkbootstrap`.

## Arduino firmware

The `arduino/clsm_control/clsm_control.ino` sketch handles incoming packets from the PC. It drives:

- A DAC (A0) and DAC MCP4725 output pair for the galvo mirrors.
- A stepper motor for Z‑axis movement with ramped speed control.
- An analog photodiode input used for autofocus and scanning feedback.

The firmware implements commands for single moves, raster scans, Z‑scans, autofocus sweeps and 3‑D scanning. Data are streamed back to the PC in big‑endian binary format.

Upload the sketch to an Arduino (tested with the R4 Minima) using the Arduino IDE or CLI.

## Running

1. Flash the Arduino with the firmware from `arduino/clsm_control`.
2. Install the Python dependencies: `pip install pyserial numpy scipy matplotlib ttkbootstrap`.
3. Connect the Arduino and run `python -m python.main -p COM4` (replace `COM4` with your serial port).

The GUI allows interactive control of the microscope and saving of scan results in NumPy `.npz` format.