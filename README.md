# EMG PCB Project

A custom wearable electromyography (EMG) acquisition system built around the **Texas Instruments ADS1299-4** analog front end and the **Seeed Studio XIAO nRF52840**.

The project includes:

- A custom 4-layer KiCad PCB
- Arduino firmware for ADS1299 configuration and sampling
- Bluetooth Low Energy (BLE) data streaming
- A Python receiver for live plotting, diagnostics, and CSV recording
- Project-local KiCad symbols, footprints, and 3D models

> **Project status:** Active prototype development. The hardware, firmware, BLE protocol, and signal-processing workflow are still being tested and refined.

---

## Project Overview

The system measures small differential biopotential signals using the ADS1299-4, transfers samples to the XIAO nRF52840 over SPI, and sends the data wirelessly to a computer over BLE.

```text
Skin electrodes
      │
      ▼
Input protection and filtering
      │
      ▼
ADS1299-4 analog front end
      │  SPI + DRDY
      ▼
XIAO nRF52840
      │  Bluetooth Low Energy
      ▼
Python receiver
      ├── Live plot
      ├── Packet diagnostics
      └── CSV recording
```

> **Add image:** photo of the assembled PCB and battery-powered

---

## Current Capabilities

- Reads the ADS1299 device ID successfully
- Configures the ADS1299 through SPI
- Supports internal test-signal and input-short testing
- Samples at **250 samples per second**
- Currently focuses on **ADS1299 Channel 4**
- Decodes signed 24-bit two's-complement ADC samples
- Streams custom BLE notifications from the XIAO nRF52840
- Receives and plots samples in Python
- Optionally records received samples to CSV
- Tracks malformed, missing, duplicate, and out-of-order packets
- Includes project-local KiCad libraries for portability

Current development work includes:

- Reducing electrode-motion artifacts
- Investigating clipping during movement
- Improving the flexed-muscle signal-to-noise ratio
- real-time data processing
- Comparing operation with and without ADS1299 BIAS drive
- Refining packet reliability and receiver-side validation
- Developing a future Android application

---

## Repository Structure

```text
EMG_PCB_PROJECT/
├── firmware/
│   ├── emg.ble/
│   │   └── emg.ble.ino
│   └── test_ads_mcu_communication_preinstal/
│       └── test_ads_mcu_communication_preinstal.ino
│
├── python/
│   └── emg_project/
│       ├── emg_ble_receiver.py
│       ├── requirements.txt
│       └── .venv/                  
│
├── hardware/
│   └── kicad/
│       └── EMG_PCB/
│           ├── EMG_PCB.kicad_pro
│           ├── EMG_PCB.kicad_sch
│           ├── ADS1299_Section.kicad_sch
│           ├── MCU_Section.kicad_sch
│           ├── Power_Supplies.kicad_sch
│           ├── EMG_PCB.kicad_pcb
│           ├── fp-lib-table
│           ├── sym-lib-table
│           ├── lib/
│           │   ├── symbols/
│           │   ├── footprints/
│           │   └── 3d_models/
│           └── EMG_PCB_Manufacture/
│
├── .gitignore
├── .gitattributes
└── README.md
```

## Hardware

### Main Components

| Component | Purpose |
|---|---|
| ADS1299-4PAGR | Four-channel, 24-bit biopotential analog front end |
| XIAO nRF52840 | Microcontroller, SPI controller, and BLE radio |
| External 2.048 MHz clock | ADS1299 master clock |
| Li-ion battery | Portable isolated power source |
| Custom analog power supplies | Generate the required analog and digital rails |
| Input protection/filter network | Protects and filters electrode inputs |
| Electrode connector | Connects the measurement and reference electrodes |

### Main Power Rails

The PCB uses the following rails:

- `+2.5 V`
- `-2.5 V`
- `3.3 V`
- Li-ion battery input
- USB `VBUS` for development and charging

The ADS1299 analog supply and reference arrangement are implemented on the custom board. The XIAO nRF52840 provides the embedded processing and BLE connection.

<img width="1056" height="810" alt="image" src="https://github.com/user-attachments/assets/d7736b43-8955-46b7-88a1-1242e41b1f84" />

<img width="1028" height="621" alt="image" src="https://github.com/user-attachments/assets/4ccff389-fa83-4c87-b4bd-8e8a7304e6db" />

---

## KiCad Design

Open the complete project using:

```text
hardware/kicad/EMG_PCB/EMG_PCB.kicad_pro
```

Do not normally open one of the child schematic files directly.

### Schematic Hierarchy

The root schematic is:

```text
EMG_PCB.kicad_sch
```

It references these hierarchical child sheets:

```text
ADS1299_Section.kicad_sch
MCU_Section.kicad_sch
Power_Supplies.kicad_sch
```

The child `.kicad_sch` files are not duplicates. Each file stores one hierarchical page used by the root schematic.

### Project-Local Libraries

Custom KiCad resources are stored in:

```text
hardware/kicad/EMG_PCB/lib/
```

This includes:

- Custom symbols
- Custom footprints
- STEP/STP 3D models
- Project library tables

Keeping these resources in the repository makes the project easier to open on another computer without losing custom parts.

### Manufacturing Files

The `EMG_PCB_Manufacture` folder contains generated Gerber and drill files. Snapshot of a specific manufactured board revision.

## Firmware

The firmware is currently developed in the Arduino IDE for the XIAO nRF52840; however, switching over to platformer.io and Cursor for serious development.

### Current Acquisition Configuration

The active configuration has included:

- 250 SPS output data rate
- Channel 4 enabled for acquisition
- Channels 1–3 powered down during single-channel testing
- 24-bit signed conversion data
- Internal square-wave test mode
- Input-short noise testing
- Optional normal electrode mode
- Experimental operation with ADS1299 BIAS disabled or enabled

Register settings may change as testing continues. The firmware source should be treated as the authoritative current configuration.

---

## BLE Data Receiver

The Python receiver is located at:

```text
python/emg_project/emg_ble_receiver.py
```

It scans for the BLE device:

```text
Nykin-EMG
```

The receiver decodes the custom BLE packet format implemented by the firmware and can:

- Plot samples live
- Print samples
- Record data to CSV
- Run for a fixed duration
- Operate without plotting
- Report packet and protocol statistics


## Development Roadmap

- [x] Design custom ADS1299/XIAO PCB
- [x] Assemble initial prototype
- [x] Verify main power rails
- [x] Read ADS1299 device ID
- [x] Capture internal test signal
- [x] Perform input-short noise testing
- [x] Stream samples over BLE
- [x] Plot and save data in Python
- [ ] Finalize safe electrode-mode register configuration
- [ ] Improve motion-artifact rejection
- [ ] Validate voltage scaling and gain calculations
- [ ] Add repeatable electrode-placement documentation
- [ ] Add digital filtering and signal-quality metrics
- [ ] Develop Android receiver application
- [ ] Complete a revised PCB based on prototype findings

---

## Author Nykin Leskiw

Developed as a personal electrical engineering and embedded-systems project
