# 🤖 OCCRA Robot Code — Official (Juggernauts Team 1)

This repository contains the official competition code for **Juggernauts Team 1** competing in the OCCRA Robotics League.

---

## 🚀 Overview & Technology Stack

The robot program includes full teleop control, autonomous routines, mechanism management, safety features, and a custom Shuffleboard dashboard. This codebase is designed for **clarity, maintainability, and reliability** during competition.

### Key Technologies

| Feature | Details |
| :--- | :--- |
| **Framework** | WPILib 2025 (Java) |
| **Architecture** | Command-based style with TimedRobot |
| **Language** | Java 17 |
| **Build System** | GradleRIO |
| **Motor Controllers** | CTRE TalonSRX and REV SparkMax |

---

## 🧰 System Requirements

### Software
* **Java 17** (Required)
    > **⚠️ Important:** WPILib 2025 does NOT support Java 18–21.
* **WPILib 2025.3.2** or later
* **VS Code** with WPILib Extension (Recommended)
* GradleRIO (Included automatically)

### Hardware
* RoboRIO (OCCRA-legal)
* CTRE TalonSRX motor controllers
* REV SparkMax motor controllers
* Sensors (encoders, limit switches, etc.)
* USB game controllers (Xbox recommended)

---

## 🤖 Robot Code Summary

### 1. 🏎️ Drivetrain System

The system uses four TalonSRX controllers with the right side inverted.

* **Driving Styles (Shuffleboard Chooser):** Arcade Drive, Tank Drive, Curvature Drive
* **Precision:** Nudge control using D-Pad for precision movements.
* **Speed Limits:** Forward capped at **70%**, Turning capped at **60%**.
* **Utility:** Automatic **180° turn** (A Button).
* **Input Smoothing:** Joystick deadband and input smoothing applied.

### 2. 🏗️ Mechanisms

| Mechanism | Controller | CAN ID | Features |
| :--- | :--- | :--- | :--- |
| **Elevator** | Spark Max | 5 | Controlled by triggers. Power capped at **50%**. Current displayed on Shuffleboard. |
| **Manipulator/Intake**| Spark Max | 6 | Controlled by Y-axis. **Intake** up to +80%; **Eject** up to –50%. |
| | | | Status updates on Shuffleboard (`INTAKE` / `OUTPUT` / `OFF`). |

### 3. 🎮 Control Modes (Shuffleboard Selectable)

* **Solo Mode:** Driver controls everything.
* **Co-Op Mode:** Driver = drivetrain, Operator = mechanisms.

### 4. 🤖 Autonomous Routines

Selectable on Shuffleboard and executed using a WPILib timer.

* **Drive Forward:** Moves robot using timed forward power.
* **Turn 180°:** Spins in place for a preset duration.

### 5. 🔧 Safety & Monitoring

* **Battery Voltage:** Low-voltage warning at `<10.5V`.
* **Current Limiting:** TalonSRX set to **40A continuous, 60A peak**.
* Monitoring of drive motor and mechanism currents.
* Safe motor defaults on startup (`disabledInit`).

---

## 📊 Shuffleboard Dashboard

Three fully programmed tabs provide essential real-time feedback and control:

1.  **Drive Tab:** Drive/Control mode choosers, real-time outputs, nudge indicator, mechanism currents, battery voltage, 180° turn status.
2.  **Autonomous Tab:** Auto mode chooser, adjustable forward-drive time, auto status, battery voltage.
3.  **Disabled Tab:** Diagnostic panel, battery voltage, and current draw statistics.

---

## 🧱 Project Architecture

The project follows the WPILib standard robot structure:

| Function | Purpose |
| :--- | :--- |
| `robotInit()` | Hardware setup |
| `robotPeriodic()`| Diagnostics & dashboard updates |
| `autonomousPeriodic()`| Timed auto steps |
| `teleopPeriodic()`| Driving & mechanism input |
| `disabledInit()` | Safe shutdown behavior |

### Vendor Libraries

Vendor dependencies are stored in the `/vendordeps/` folder:

* REV SparkMax
* CTRE Phoenix

---

## 🙌 Acknowledgements

Special thanks to the programmers, OCCRA organizers, WPILib developers, and REV Robotics & CTRE engineers for their support.
