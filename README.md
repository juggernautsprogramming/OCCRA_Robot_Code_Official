# 🤖 OCCRA Robot Code — Official (Juggernauts Team 1)

This repository contains the official, single-file Java control code (`Robot.java`) for the **Juggernauts Team 1** robot competing in the OCCRA Robotics League.

The code base is designed for **maximum reliability and rapid deployment** during competition. It is built upon the robust **WPILib TimedRobot** framework, utilizing a clean structure to manage hardware configuration, safety protocols, and complex control logic.

## ⚙️ Core Technology & Architectural Decisions
---

### Framework & Language
The project uses **WPILib 2025** and is programmed in **Java 17**. The choice of Java 17 is crucial, as the 2025 WPILib version specifically supports this long-term support (LTS) version, providing stability and predictable performance on the RoboRIO.

### Architectural Choice: TimedRobot
Instead of a complex, layered Command-based architecture, we opted for the simpler **TimedRobot** model. This choice guarantees predictable execution timing and ensures all control loops are managed within the standard periodic functions (`robotPeriodic`, `teleopPeriodic`, etc.), minimizing overhead and making on-the-fly debugging easier during a short OCCRA match day.

| Feature | Detail |
| :--- | :--- |
| **Language** | Java 17 |
| **Framework** | WPILib 2025 (TimedRobot) |
| **Drivetrain Controllers** | 4x CTRE TalonSRX (CAN IDs 1-4) |
| **Mechanism Controllers** | 2x REV SparkMax (CAN IDs 5 & 6) |
| **Control Interface** | 2x Xbox Controllers (Driver & Operator) |

---

## 🏎️ Drivetrain System Deep Dive

The drivetrain is a four-motor system controlled via two **WPI_TalonSRX** masters and two followers.

### Safety and Configuration
The drive system is configured with several critical safety features in `robotInit()`:

* **Motor Inversion:** The right side motors are inverted to ensure that all motors drive forward with a single positive command input.
* **Follower Configuration:** The rear motors (CAN IDs 2 & 4) are correctly set as followers to the front masters (CAN IDs 1 & 3), simplifying control via the `DifferentialDrive` utility.
* **Brake Mode:** The `setNeutralMode(NeutralMode.Brake)` command is executed at startup and autonomous initialization to provide instant stopping power, critical for maneuverability and precision.
* **Current Limiting (Protection):** To protect the motors and the robot's electrical system, a robust **Supply Current Limit** is configured on all four TalonSRX controllers: **40A Continuous** and **60A Peak** (with a 0.1-second surge time).

### Hierarchical Drive Control (`teleopPeriodic`)

Drive inputs are processed in a strict hierarchy to ensure safety and responsiveness:

1.  **Level 1: D-Pad Nudge (Highest Priority)**
    * If the Driver's POV (D-Pad) is pressed, the robot executes a precision move at a slow **25% power (`NUDGE_SPEED`)**. This logic immediately **overrides and bypasses** all other joystick inputs, providing excellent fine control for lining up.
2.  **Level 2: 180° Turn Macro (A Button)**
    * Activated by the Driver's A button, a state machine controls a time-based turn (`TURN_TIME` is currently 1.69s). This also takes precedence over normal joystick control while active, allowing the driver to quickly reorient the robot.
3.  **Level 3: Joystick Drive (Primary Control)**
    * **Input Scaling:** All joystick inputs are capped using **70% speed (`SPEED_SCALE`)** and **60% turn (`TURN_SCALE`)** to prevent the robot from being uncontrollable at full stick deflection.
    * **Input Smoothing (Intermediate Power):** **Deadband (0.1)** is applied to eliminate joystick drift. To achieve a balance between precision and speed, all drive inputs are smoothed using the **$x^{1.75}$ power curve** (`Math.pow(abs(input), 1.75)`). This provides fine control near the center while quickly ramping up to maximum speed.
    * **Drive Modes:** The code correctly routes inputs for three user-selectable modes: `Arcade`, `Tank`, and `Curvature`.

---

## 🏗️ Mechanism Control Breakdown

All mechanism control uses proportional output based on controller inputs, with mechanism-specific speed limits enforced in code.

### Elevator Subsystem (CAN 5)
* **Controller Class:** Uses the modernized `SparkMax` class (replacing the deprecated `CANSparkMax`).
* **Control:** Uses the **Operator's or Driver's Triggers** (depending on Control Mode).
* **Proportional Speed:** The difference between the Left and Right Trigger axes determines the direction and speed.
* **Speed Limit:** The final output is capped at **50% (`ELEVATOR_MAX_SPEED`)** to prevent excessive motor strain and provide smooth lifting/lowering.

### Manipulator / Intake (CAN 6)
* **Control:** Uses the **Mechanism Controller's Right Y-Axis**.
* **Asymmetrical Power:** The code enforces different maximum speeds for intake and output, reflecting different power requirements:
    * **Intake (Stick Forward):** Scaled up to **+80% (`INTAKE_SPEED`)** for powerful collection.
    * **Eject/Output (Stick Backward):** Scaled to **-50% (`OUTPUT_SPEED`)** for controlled scoring.
* **Status Reporting:** The `manipulatorStatusEntry` displays the current action and actual power level on the Shuffleboard dashboard.

---

## 🎮 Operational Modes and Autonomous

### Control Modes
The `Control Mode Chooser` widget allows teams to quickly switch between staffing configurations:

* **"Solo Mode":** Driver (Port 0) manages all movement and mechanism functions.
* **"Co-Op Mode":** Driver (Port 0) manages only Drivetrain; Operator (Port 1) manages all Mechanisms.

### Autonomous Routines
Autonomous relies on the simple, reliable WPILib `Timer` class for time-based execution. **Note: These routines are subject to environmental variations (battery, carpet) and should be validated before each match.**

* **Routines:** `Drive Forward` and `Turn 180°`.
* **Configuration:** The duration for the `Drive Forward` routine is configurable via the `Auto Drive Time (s)` entry on the Autonomous tab.

---

## 📊 Comprehensive Shuffleboard Dashboard & Diagnostics

The robot program includes a fully wired-up Shuffleboard dashboard across three dedicated tabs, using specific widgets for clear data presentation.

### Live Diagnostic Data (`robotPeriodic`)
The `robotPeriodic()` function continuously updates critical data:

* **Drive Outputs:** Displays motor output percentages for Left and Right sides.
* **Current Monitoring:** Calculates and displays aggregated current draw for the Left Drive, Right Drive, Elevator, and Manipulator.

### Safety and Warning System
* **Battery Voltage Warning:** A universal check triggers a highly visible warning on both the **Drive Tab** and the **Disabled Tab** if the main battery voltage drops **below 10.5V**.
* **Disabled State Check (`disabledInit`):** When the robot disables, it checks the **Total Current Draw** across the PDH. If the draw is above 5.0A, a warning is logged to the `Disabled Tab`, alerting the team to potential short circuits or unexpected motor behavior before the next match.

### Tab Contents Summary

| Tab Name | Key Functionality | Example Widgets Used |
| :--- | :--- | :--- |
| **Drive** | Primary match-time interface. Shows live outputs, current health, and mode choosers. | Combo Box Chooser, Voltage View (for current), Dial, Number Bar. |
| **Autonomous** | Pre-match configuration and status monitoring for autonomous routines. | Sendable Chooser, Text View, Voltage View (for battery). |
| **Disabled** | Post-match diagnostics and safety checks. | Text View (for current check results), Battery Status Warning. |

---

## 💻 V. Essential Code Structure & Constants

To allow next year's team to quickly adjust the robot's performance parameters, the most critical configuration values are defined as `private static final double` constants at the top of the `Robot.java` file.

### A. Tuning and Safety Constants

| Constant Name | Value Type | Current Value | Purpose |
| :--- | :--- | :--- | :--- |
| `DEADBAND` | `double` | 0.1 | Input threshold to ignore joystick drift. |
| `SPEED_SCALE` | `double` | 0.70 | Maximum forward/reverse speed (0.0 to 1.0). |
| `TURN_SCALE` | `double` | 0.60 | Maximum rotational speed (0.0 to 1.0). |
| `SHAPE_EXPONENT` | `double` | 1.75 | The power curve exponent for input smoothing (e.g., 2.0 = squaring, 3.0 = cubing). |
| `NUDGE_SPEED` | `double` | 0.25 | Fixed speed for D-Pad precision control. |
| `TURN_TIME` | `double` | 1.69 (s) | Calibrated duration of the 180° Turn Macro. **Requires field testing.** |
| `VOLTAGE_MIN_WARN` | `double` | 10.5 (V) | Battery voltage threshold to trigger dashboard warning. |
| `ELEVATOR_MAX_SPEED` | `double` | 0.50 | Maximum output for the Elevator motor. |
| `INTAKE_SPEED` | `double` | 0.80 | Maximum output for the Manipulator intake action. |
| `OUTPUT_SPEED` | `double` | 0.50 | Maximum output for the Manipulator eject/output action. |

### B. CAN ID Mapping

These constants map the physical devices to the specific CAN network addresses on the robot. **These must match the configuration set on the motor controllers themselves!**

| Constant Name | CAN ID | Device | Subsystem |
| :--- | :--- | :--- | :--- |
| `CAN_ID_LEFT_FRONT` | 1 | TalonSRX (Master) | Drivetrain (Left) |
| `CAN_ID_LEFT_REAR` | 2 | TalonSRX (Follower) | Drivetrain (Left) |
| `CAN_ID_RIGHT_FRONT` | 3 | TalonSRX (Master) | Drivetrain (Right) |
| `CAN_ID_RIGHT_REAR` | 4 | TalonSRX (Follower) | Drivetrain (Right) |
| `CAN_ID_ELEVATOR` | 5 | SparkMax | Elevator |
| `CAN_ID_MANIPULATOR` | 6 | SparkMax | Manipulator |

---

## 🐛 VI. Troubleshooting & Debugging Guide

This section helps diagnose common issues, focusing on the specific architecture of this single-file code base.

### 1. Drivetrain is Moving Slowly or Not at All

| Symptom | Diagnosis | Fix |
| :--- | :--- | :--- |
| Robot moves, but is slow. | `SPEED_SCALE` / `TURN_SCALE` is too low, or `SHAPE_EXPONENT` is too high (e.g., 3.0). | Check and increase `SPEED_SCALE` / `TURN_SCALE`. Reduce `SHAPE_EXPONENT` to $1.5$ or $1.75$. |
| Motors are hot, low speed. | **Current Limiting** is engaging too often. | Check the Drive Tab's current monitoring. If current is near 40A continuously, slightly increase the `Supply Current Limit` on the TalonSRX configurations, or reduce `SPEED_SCALE`. |
| Only one side moves. | **Follower** not set correctly. | Verify the `follow()` command for the rear motors is executed correctly in `robotInit()`. |
| D-Pad Nudge moves robot backward. | The `NUDGE_SPEED` value needs to be inverted in the code, or the arcade drive parameters are swapped. | Check the `teleopPeriodic()` D-Pad switch-case logic and signs. |

### 2. Mechanism Control Issues

| Symptom | Diagnosis | Fix |
| :--- | :--- | :--- |
| Motor moves, but too fast/slow. | Mechanism-specific speed cap is incorrect. | Adjust `ELEVATOR_MAX_SPEED`, `INTAKE_SPEED`, or `OUTPUT_SPEED` constants. |
| Manipulator runs backward. | Motor is inverted incorrectly for the code's intended direction. | Check the motor controller's inversion setting in `robotInit()`. Toggle the `setInverted()` parameter (e.g., from `true` to `false`). |
| Elevator only moves one way. | One of the two trigger inputs is not being read correctly by the controller/code. | Check controller bindings for the Left/Right Trigger Axes on the operator controller. |

### 3. Dashboard and Console

* **"I can't see the warnings!"** Ensure your driver station computer is running the **Shuffleboard** program and that the correct `Drive`, `Autonomous`, and `Disabled` tabs are selected.
* **Console Warnings:** Always check the **DS Log Viewer** for **yellow or red text** during initialization (`robotInit`). Failure to configure a CAN ID (e.g., a device is unplugged) will typically result in a warning here.