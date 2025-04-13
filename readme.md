# ESP32 Jet ECU Wiki Documentation

This firmware is designed for an ESP32-based engine control unit (ECU) that manages engine startup, ignition, oil pressure regulation, throttle control, and various safety systems. The ECU reads multiple sensor inputs, performs dynamic idle adjustments, and communicates over both a secondary serial channel (for an instrument cluster) and via Bluetooth. This document explains the hardware setup, software structure, and detailed function-by-function reference.

---

## Table of Contents

- [Overview](#overview)
- [Hardware Setup & Pin Assignments](#hardware-setup--pin-assignments)
- [Software Architecture](#software-architecture)
  - [Global Configuration & Flags](#global-configuration--flags)
  - [Main Control Flow](#main-control-flow)
- [Detailed Function Reference](#detailed-function-reference)
  - [Setup and Loop](#setup-and-loop)
  - [Engine Start-up Sequence](#engine-start-up-sequence)
  - [Engine Shutdown and Cooldown](#engine-shutdown-and-cooldown)
  - [Sensor Readings & Data Processing](#sensor-readings--data-processing)
  - [Actuator Control Functions](#actuator-control-functions)
  - [Communication & Command Handling](#communication--command-handling)
  - [Watchdog and ADC Functions](#watchdog-and-adc-functions)
  - [Calibration & Averaging Utilities](#calibration--averaging-utilities)
- [Command Reference](#command-reference)
- [Notes and Future Improvements](#notes-and-future-improvements)

---

## Overview

The ESP32 Jet ECU firmware implements an engine control system with multiple functions including:
- **Startup and Ignition Sequence:** Gradual starter control, oil feed adjustments, and flame detection.
- **Dynamic Idle Control:** Adaptive throttle management based on idle input, current RPM, and target idle RPM.
- **Sensor Monitoring:** Continuous acquisition of temperature (via a MAX6675 thermocouple module), RPM (using the ESP32’s PCNT peripheral), oil pressure (via ADC readings), and flame detection.
- **Safety and Watchdog Protection:** Automatic shutdown for overspeed, under-speed, and over-temperature situations.
- **User Command Interface:** Bluetooth commands for calibration, manual overrides, and testing routines.
- **Persistent Calibration:** EEPROM storage and retrieval of throttle calibration values.

---

## Hardware Setup & Pin Assignments

The firmware configures many pins for its operation. Key assignments include:

- **Serial Communication:**
  - **Serial2:** Configured for instrument cluster communications (TX2 on GPIO17, RX2 on GPIO16).
  - **Bluetooth Serial (SerialBT):** Used for debugging, logging, and command input.
  
- **Input Pins:**
  - `idlePin (GPIO33)`: Reads idle potentiometer for throttle calibration.
  - `StopSwitch (GPIO15)` & `StartSwitch (GPIO13)`: Manual engine stop/start switches.
  - `hallSensorPin (GPIO14)`: Reads engine RPM via a hall-effect sensor.
  - `throttlePotPin (GPIO32)`: Measures throttle position.
  - `flameDetectorPin (GPIO35)`: Monitors flame (spark/ignition) using analog readings.
  - `oilPressureSensorPin (GPIO34)`: Reads oil pressure data.
  
- **Output Pins:**
  - `ledPin (GPIO2)`: Onboard LED for status indication.
  - `pwmPin (GPIO4)`: PWM output to control the oil pump.
  - `igniterPin (GPIO21)`: Controls igniter MOSFET.
  - `starterPin (GPIO23)`: Drives the starter ESC.
  - `throttlePin (GPIO22)`: Controls the throttle ESC.
  - `EscActivationPin (GPIO25)`: Used to enable/disable the starter ESC.
  
- **MAX6675 Thermocouple Module Pins:**
  - `thermoCLK (GPIO5)`, `thermoCS (GPIO18)`, and `thermoDO (GPIO19)`

Additional unused or reserved pins are set to known states during setup to prevent erratic behavior.

---

## Software Architecture

### Global Configuration & Flags

The firmware uses a number of defined constants and global flags to control behavior:

- **RPM Limits & Temperature Thresholds:**  
  - `MIN_RPM`, `TGT_RPM`, and `RPM_LIMIT` are used to ensure the engine is operating within safe speeds.
  - `TOT_THRESHOLD` defines the maximum allowable temperature.
  
- **Timing Variables:**  
  - `READ_INTERVAL` governs how often sensor data is refreshed.
  - Delay values and iteration counts are embedded in routines like `sleep()` and oil pressure adjustments.
  
- **Mode Flags:**  
  - Flags such as `skipSafetyChecks`, `dynamicIdleEnabled`, `enableSensorData`, and `disableStarterWhenRunning` let users override or toggle key safety and performance features during testing or operation.

### Main Control Flow

- **`setup()` Function:**  
  Performs initialization of serial communications, pin modes, ADC calibration, PCNT (pulse counter) configuration, EEPROM retrieval, watchdog setup, and attachment of ESC servo objects. It then sets the device to a safe idle state while performing an idle calibration.

- **`loop()` Function:**  
  The core loop toggles the status LED and continually updates sensor data (via `readAndReportTemperature()`), monitors safety conditions, and processes user input (when not running). When the engine is active (`runningState` is true), it executes throttle and oil feed management and monitors for overspeed/low RPM conditions.

---

## Detailed Function Reference

### Setup and Loop

- **`setup()`**  
  - **Purpose:** Configures all hardware interfaces (GPIO, serial, ADC, PCNT, EEPROM, servos) and initializes the system.
  - **Actions:**
    - Starts Bluetooth and secondary serial communications.
    - Configures pin modes and initial states.
    - Loads throttle calibration values from EEPROM.
    - Initializes ADC channels and pulse counter for RPM.
    - Sets initial safe states for oil pump, throttle, and starter.
  
- **`loop()`**  
  - **Purpose:** Main execution loop; updates sensor data, maintains watchdog resets, processes engine running conditions, and listens for commands when idle.
  - **Behavior:**  
    - Toggles LED as a heartbeat indicator.
    - Updates temperature and RPM every `READ_INTERVAL` milliseconds.
    - Checks for safety conditions (low/high RPM, over-temperature) and invokes shutdown if needed.
    - If the engine is off (`!runningState`), monitors switches and listens for Bluetooth commands.

### Engine Start-up Sequence

- **`startup()`**  
  - **Purpose:** Implements the engine starting sequence.
  - **Steps:**
    - Notifies via serial channels and enables the starter ESC.
    - Reads idle throttle value from ADC and calculates a starting throttle position.
    - Adjusts oil pressure using `setOilValue()` and gradually ramps up the starter ESC.
    - Activates the igniter once RPM thresholds are met; monitors for successful flame detection.
    - On successful ignition, the engine enters running state and dynamic idle control is enabled.
  
### Engine Shutdown and Cooldown

- **`shutdown()`**  
  - **Purpose:** Safely stops engine operation and reboots the ESP32.
  - **Behavior:**
    - Disables the watchdog timer.
    - Sets throttle, starter ESC, and oil pump to safe positions.
    - Initiates a cooldown sequence—if running—by maintaining oil feed until RPM falls below threshold.
    - Uses a final starter activation to cool the turbine before restarting the ESP32.

### Sensor Readings & Data Processing

- **`readAndReportTemperature()`**  
  - **Purpose:** Reads the engine temperature using the MAX6675 thermocouple.
  - **Actions:**  
    - Validates temperature readings and updates the TOT (Temperature Over Time) variable.
    - Calls `calculateRPM()` to compute and average engine RPM.
    - Transmits temperature and RPM data to both the instrument cluster and Bluetooth.
    - Invokes oil pressure adjustments and checks for over-temperature shutdown conditions.

- **`flameDetector()`**  
  - **Purpose:** Determines if ignition or flame is present.
  - **Mechanism:**  
    - Reads the flame sensor via ADC and compares the average to a predefined threshold.

- **`readPressure()`**  
  - **Purpose:** Reads an analog value from the oil pressure sensor and converts it into a pressure (in bars) using a quadratic approximation.

### Actuator Control Functions

- **`setOilRpm(int oilPercentage)`**  
  - **Purpose:** Generates a PWM signal on the oil pump output pin.
  - **Implementation:**  
    - Maps a given percentage value to a PWM duty cycle and updates the oil pump accordingly.

- **`setOilValue(float oilPressureTarget, bool singleAdjustment)`**  
  - **Purpose:** Dynamically adjusts the oil pump’s output to reach a desired oil pressure.
  - **Logic:**  
    - Continuously reads oil pressure, computes the difference to the target, and adjusts the pump speed incrementally.
    - Performs a fixed number of iterations before terminating if the target isn’t reached.

- **`setStarterESC(int targetSetting, float accelerationFactor)`**  
  - **Purpose:** Controls the starter ESC servo by gradually transitioning from the current setting to the target.
  - **Details:**  
    - Uses a step delay (computed from an acceleration factor) to provide a smooth change.
    - Monitors sensor data during the transition and checks for a stop condition.

- **`updateThrottleESC(int requestedValue)`**  
  - **Purpose:** Generates throttle control signals based on either automatic sensor inputs or a direct value.
  - **Features:**
    - When in automatic mode (requestedValue is 1), it reads throttle and idle potentiometers.
    - Implements dynamic idle adjustment using a rolling average of RPM and timing of idle events.
    - Contains safeguards to lower throttle output if RPM or temperature exceed safe limits.
    - Allows manual override when requested.

### Communication & Command Handling

- **`listenForCommands()`**  
  - **Purpose:** Monitors the Bluetooth serial port for incoming commands.
  - **Supported Commands:**  
    - **FuelPrime:** Executes a fuel priming routine.
    - **IGNtest:** Runs an ignition test by activating the igniter.
    - **StartTest:** Tests the starter ESC and oil feed.
    - **OilPrime:** Activates the oil pump for priming.
    - **StartUp:** Manually initiates the startup sequence.
    - **ToggleSafetyChecks / ToggleSensorData / ToggleIdleMode:** Toggles various features.
    - **SetOilPressure / SetOil:** Adjusts oil pump settings based on numeric input.
    - **PotRead:** Reads and reports the current throttle and idle potentiometer values.
    - **ThrottleCalibration:** Runs a calibration routine to save throttle position values to EEPROM.

- **EEPROM Functions:**
  - **`saveCalibrationValues(int minThrottleValue, int maxThrottleValue)`**  
    - Saves throttle calibration data along with a checksum and validation byte.
  - **`loadCalibrationValues()`**  
    - Retrieves calibration data from EEPROM and validates it before applying.

### Watchdog and ADC Functions

- **Watchdog Control:**  
  - **`resetWatchdog()`, `disableWatchdog()`, `enableWatchdog()`**  
    - These functions interface with the ESP32’s task watchdog to ensure system responsiveness.
  - **`setupWatchdog()`**  
    - Initializes the watchdog timer with a defined timeout and core mask for dual-core monitoring.

- **ADC Configuration:**  
  - **`setupADC()`**  
    - Sets the ADC width and attenuation for channels on ADC1 (GPIOs 32–39), and characterizes these channels for accurate voltage measurement.
  - **`getAverageADC(int pin, int samples)`**  
    - Performs multiple ADC readings on a given pin and returns their average. This is used to smooth sensor data.

### Calibration & Averaging Utilities

- **`calibrateMinThrottle()`**  
  - **Purpose:** Calibrates the minimum throttle position during startup using multiple ADC samples from the throttle potentiometer.
  - **Outcome:**  
    - Adjusts the global `minThrottleValue` if enough valid samples are obtained; otherwise, forces a reboot (if configured).

- **`RPMAverage(int newRPMValue)`** & **`TOTAverage(float newTOTValue)`**  
  - **Purpose:** Maintain rolling averages for engine RPM and temperature to smooth out transient sensor noise.
  - **Implementation:**  
    - Uses fixed-size circular buffers to compute the average only from valid readings.

- **`sleep(int duration)`**  
  - **Purpose:** Implements a long delay routine that, during pauses, continues to update sensor readings and check for stop conditions.
  - **Usage:**  
    - Often called during startup, shutdown, or waiting periods in command routines.

- **`calculateRPM()`**  
  - **Purpose:** Uses the ESP32’s pulse counter (PCNT) to compute engine RPM from hall sensor pulses.
  - **Method:**  
    - Reads the current pulse count, calculates the time difference between readings, and updates the rolling average via `RPMAverage()`.

- **`relight(unsigned long currentTime)`**  
  - **Purpose:** Attempts to reignite the engine if a flameout is detected or if RPM falls below a threshold.
  - **Behavior:**  
    - Monitors the Start switch and flame sensor, temporarily activating the igniter and adjusting the starter accordingly.

---

## Command Reference

When connected via Bluetooth, the ECU listens for the following commands (each command should be terminated by a newline):

- **FuelPrime:** Prime the fuel pump by adjusting the throttle ESC temporarily.
- **IGNtest:** Test ignition by activating the igniter for a short duration.
- **StartTest:** Initiate a starter test (activate starter ESC and oil pump).
- **OilPrime:** Activate the oil pump for 10 seconds at a predetermined percentage.
- **StartUp:** Begin the full engine startup sequence (if the Stop switch is not active).
- **ToggleSafetyChecks:** Toggle safety checks on/off (bypassing certain limits during testing).
- **ToggleIdleMode:** Switch between dynamic idle adjustment and static idle control.
- **ToggleSensorData:** Enable/disable the continuous sensor data feed over Bluetooth.
- **SetOilPressure x.x:** Set a target oil pressure (in bars) for a short period.
- **SetOil xx:** Set the oil pump speed to a specific percentage.
- **PotRead:** Read and display the current potentiometer values for throttle and idle.
- **ThrottleCalibration:** Run the throttle calibration routine; valid calibration data is saved to EEPROM.

---

## Notes and Future Improvements

- **Safety Enhancements:** Although several safety checks are implemented (such as low RPM cutoffs and over-temperature shutdown), additional redundancy and error handling could be added.
- **Modularization:** Breaking the firmware into smaller libraries or modules may improve maintainability.
- **Advanced Diagnostics:** Future revisions may include more detailed logging and diagnostic output for debugging and performance monitoring.
- **User Interface:** Enhancements to the Bluetooth command set could include more robust error reporting and interactive calibration steps.
