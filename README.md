# Avalon - Firmware

## Overview

This repository contains the firmware for **Avalon**, a supercapacitor power management module developed by the Calibur Robotics team at NUS for the RMUC 2025 competition.

Avalon is designed to manage power flow between the robot’s battery and an onboard supercapacitor bank. The goal is to provide temporary power bursts to the chassis during intense maneuvers — such as sprints, turns, or impacts — without violating the strict continuous power limits imposed by the RoboMaster referee system.

To achieve this, the firmware operates a bidirectional synchronous buck-boost converter, driven by an STM32G4 microcontroller. The firmware monitors voltages and currents in real-time using filtered ADC measurements, and runs multiple cascaded PID controllers to regulate charging and discharging cycles efficiently and safely. When the robot is idle or cruising, excess power is used to charge the supercaps; during high-demand events, stored energy is discharged into the system to supplement the chassis power.

The firmware is developed in **STM32CubeIDE**, uses **FreeRTOS**, and includes custom modules for ADC filtering, control logic, CAN communication, safety handling, and gate signal generation through HRTIM.


## Folder Structure

The project is organized to separate the core firmware logic from hardware abstraction and utility layers. 
The structure below only includes the essential files for control, communication, and signal processing logic. Other files (e.g. peripheral initialization, HAL drivers, and CubeMX-generated boilerplate) are omitted for clarity.
```bash
.
Core/
└── Src/
    ├── main.cpp             # Firmware entry point: initializes system and peripherals
    └── app_freertos.cpp     # Sets up FreeRTOS tasks and scheduling

System/
└── stm_utils/
    └── Src/
        ├── supercap_controllers.cpp/.hpp   # Main control loop, PID regulators, safety state machine
        ├── adc_utils.cpp/.hpp              # ADC DMA sampling, low-pass filters, calibration offsets
        ├── msg_handler.cpp/.hpp            # CAN communication: command reception and telemetry messages
        └── supercap_def.h                  # Configuration header for gains, thresholds, safety limits

```

## Logic Flow

The firmware follows a cyclic real-time control strategy driven by the **HRTIM Repetition Event Callback**, which is called periodically and serves as the main execution loop.

1. **Startup & Initialization**
   - All peripherals (ADC, HRTIM, CAN, DMA) are configured via STM32CubeMX.
   - PID controllers are initialized.
   - The system waits for a valid initial duty cycle based on voltage ratio between both gates of the buck/boost converter before enabling gate PWM.

2. **ADC Sampling**
   - ADC1 samples capacitor-side voltages and currents; ADC2 samples the battery-side.
   - DMA collects 16 samples per channel, and a moving average filter smooths them.
   - Measurements are compensated using calibrated gain/offset values.

3. **Main Control Loop (Triggered by HRTIM Callback)**
   - The loop proceeds as follows:
     - Update voltage, current, and power measurements.
     - Run PIDs for output power, then infer a reference current.
     - Apply feed-forward estimation for capacitor ESR.
     - Bound the duty cycle to avoid capacitor overcharge/discharge.
     - Write the computed duty ratio to the PWM registers via HRTIM.

4. **Safety Watchdogs**
   - Independent voltage and current monitors track each measurement.
   - If any metric exceeds predefined safe bounds (with hysteresis), the loop is stopped and the gates are disabled until recovery.

5. **CAN Message Handling**
   - On receiving a new control packet, the firmware will:
     - Update the power limit and enable state.
     - Reset or re-enable the loop as needed.
     - Adjust the internal power reference.

## Communication

The firmware communicates with the rest of the robot system using **CAN** via the `FDCAN3` peripheral of the STM32G4. All interaction between the supercapacitor power manager and the robot’s central controller is handled through this interface.

Two custom CAN packets are defined:

- A **Receive packet (RX)** that allows the robot's main controller to enable or disable the module, set the maximum power limit and trigger a software reset.
- A **Transmit packet (TX)** that sends back the current power draw and supercapacitor charge level, allowing the rest of the system to make informed decisions based on the energy status.



### Receive Packet (RX)

| Field           | Type      | Description                                                        |
|-----------------|-----------|--------------------------------------------------------------------|
| `enable_module` | `uint8_t` | Enables (`1`) or disables (`0`) the power management loop          |
| `reset`         | `uint8_t` | Triggers a software reset when set to `1`                          |
| `pow_limit`     | `uint8_t` | Maximum power the chassis is allowed to consume (in watts)         |
| `energy_buffer` | `uint16_t`| Placeholder to tune future dynamic behaviors based on energy state |

### Transmit Packet (TX)

| Field            | Type      | Description                                                       |
|------------------|-----------|-------------------------------------------------------------------|
| `chassis_power`  | `float`   | Measured power draw of the chassis (voltage × current)            |
| `error`          | `uint8_t` | Reserved for future fault flags or error reporting (currently 0)  |
| `cap_energy`     | `uint8_t` | Estimated supercapacitor charge level (mapped from 0 to 255)      |

## Build & Flash Instructions

This project is developed in **STM32CubeIDE** and is intended to run on an STM32G4 series microcontroller (specifically, STM32G474RE). To build and flash the firmware:

1. Open STM32CubeIDE and import this project as an existing STM32Cube project.
2. Make sure the correct target board and debugger (e.g. ST-Link) are selected.
3. Connect the board with your ST-LINK.
4. Click **File > Open Projects from File System**.
5. Click **Directory** and choose the folder where the project is, then click **Finish**.
5. Click **Run > Debug As > STM32 Cortex-M C/C++ Application** to flash the firmware to the board.

The project uses FreeRTOS. The default configuration assumes DMA-driven ADC sampling, HRTIM timers for gate signal generation, and CAN communication on FDCAN3.

If you're using a different STM32G4 variant, make sure to update the pin configuration accordingly in `.ioc`.

## Documentation

This project is documented using **Doxygen**. You can generate and view the documentation locally, or access it online via our [GitHub Pages site](https://nusrobomaster.github.io/Avalon_FW/).

## License

This project is released under the **MIT License**.

See [`LICENSE`](LICENSE) for full details.
