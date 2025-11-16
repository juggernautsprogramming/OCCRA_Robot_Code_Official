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

1.  **Level 1: D-Pad Nudge (Highest Priority)**
    * If the Driver's POV (D-Pad) is pressed, the robot executes a precision move at a slow **25% power (`NUDGE_SPEED`)**. This logic immediately **overrides and bypasses** all other joystick inputs, providing excellent fine control for lining up.
2.  **Level 2: 180° Turn Macro (A Button)**
    * Activated by the Driver's A button, a state machine controls a 1.0-second timed turn. This also takes precedence over normal joystick control while active, allowing the driver to quickly reorient the robot.
3.  **Level 3: Joystick Drive (Primary Control)**
    * **Input Scaling:** All joystick inputs are capped using **70% speed (`SPEED_SCALE`)** and **60% turn (`TURN_SCALE`)** to prevent the robot from being uncontrollable at full stick deflection.
    * **Input Smoothing:** **Deadband (0.1)** is applied to eliminate joystick drift, and the turn input is **squared** (`Math.copySign(turn * turn, turn)`) to give the driver finer, lower-speed control around the center point.
    * **Drive Modes:** The code correctly routes inputs for three user-selectable modes: `Arcade`, `Tank`, and `Curvature`.

---

## 🏗️ Mechanism Control Breakdown

All mechanism control uses proportional output based on controller inputs, with mechanism-specific speed limits enforced in code.

### Elevator Subsystem (CAN 5)
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
Autonomous relies on the simple, reliable WPILib `Timer` class for time-based execution.

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
