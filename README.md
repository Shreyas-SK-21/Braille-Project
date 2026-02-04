# Touch-Based Pressure & Tracking System (Velostat Matrix)

## Overview

This project implements a **touch and pressure tracking system** using a **Velostat-based resistive matrix**, multiplexers, and an Arduino-class microcontroller. The system scans a grid of pressure-sensitive cells, dynamically calibrates noise, detects touch events reliably, and streams structured data for visualization and analysis on a PC.

The long-term motivation is to build a **robust tactile sensing platform** that can be extended toward applications like **Braille reading assistance, gesture tracking, pressure heatmaps, and human–computer interaction research**.

---

## What This Repository Contains

* Firmware for scanning an **8×8 resistive sensor matrix**
* Dynamic **baseline and noise calibration**
* Touch detection with **adaptive thresholds**
* CSV-style serial data streaming
* PC-side plotting and analysis tools (Python)
* Documentation of **engineering problems faced and solutions implemented**

---

## Hardware Setup

### Components Used

* Velostat sheet (pressure-sensitive resistive material)
* Copper tape / copper strips (row & column electrodes)
* Arduino-compatible microcontroller
* Analog multiplexer(s) for row/column scanning
* Resistors (notably 100kΩ for stability)
* Jumper wires, breadboard / custom backing

### Matrix Structure

* Physical sensor matrix: **8 rows × 8 columns**
* Logical interpretation layer: **grid mapping for tracking & heatmaps**

Each intersection of a row and column forms a pressure-sensitive cell whose resistance changes under touch.

---

## System Architecture

1. **Row/Column Selection**
   Multiplexers activate one row and one column at a time.

2. **Analog Sampling**
   The voltage at the intersection is read via the ADC.

3. **Baseline Calibration**
   Initial no-touch values are learned and stored.

4. **Noise Profiling**
   The system records peak-to-peak noise to set safe thresholds.

5. **Touch Detection**
   A touch is detected when the signal crosses an adaptive threshold above noise.

6. **Tracking & Mapping**
   Valid touches are mapped into grid coordinates and optionally smoothed.

7. **Data Output**
   Data is streamed over serial in CSV format for logging and plotting.

---

## Firmware Features

* Dynamic noise learning window (time-based)
* Per-cell baseline tracking
* Protection against false positives
* Stable scanning without burst noise
* Configurable thresholds and timing constants

---

## Data Output Format

The firmware outputs structured serial data in **CSV-compatible format**, enabling:

* Offline logging
* Python-based plotting
* Velocity, reversal, and zero-crossing analysis

Example:

```
timestamp,row,col,raw_value,delta
```

---

## Visualization & Analysis

Python scripts are used to:

* Plot raw pressure values
* Generate heatmaps
* Detect motion trends (velocity & reversals)
* Map data to a Braille-like 4×8 logical structure

---

## Problems Faced & Solutions

> This section documents real engineering issues encountered during development and how they were resolved.

### 1. False Touches with No Contact

**Problem:** Random touches were detected even when the surface was untouched.

**Cause:**

* Floating ADC values
* Insufficient noise margin
* Over-aggressive thresholds

**Solution:**

* Added long-duration noise learning
* Increased resistance values for stability
* Introduced adaptive thresholds based on measured noise

---

### 2. No Output After Calibration Changes

**Problem:** Touch detection stopped working entirely.

**Cause:**

* Thresholds became higher than actual signal deltas

**Solution:**

* Logged intermediate values
* Tuned calibration windows
* Separated noise calibration from touch calibration

---

### 3. Continuous Touch Instead of Bursts

**Problem:** Touches were detected continuously rather than as distinct events.

**Cause:**

* Baseline not updating correctly
* Slow decay in signal processing

**Solution:**

* Introduced controlled baseline drift handling
* Added debounce-like logic

---

*(This section is intentionally expandable as development continues.)*

---

## Current Project Status

* Core scanning and detection: ✅ Implemented
* Noise handling & calibration: ✅ Stable
* Serial data streaming: ✅ Working
* Visualization tools: ✅ Functional
* Extended gesture interpretation: 🔄 In progress

---

## Future Improvements

* Higher resolution matrix
* Hardware filtering
* Interrupt-based scanning
* Machine learning–based touch classification
* Dedicated PCB design

---

## How to Use

1. Assemble the Velostat matrix
2. Upload the firmware to the microcontroller
3. Open serial monitor / logger
4. Run Python scripts for visualization

---

## Project Philosophy

This project emphasizes:

* **Engineering robustness over hacks**
* **Measurable noise-aware design**
* **Iterative debugging with real data**

---

## Author

Developed as an exploratory hardware–software co-design project.

---

## License

(To be decided)