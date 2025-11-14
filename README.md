OCCRA Robot Code — Official

by Juggernauts Team 1 Programming

🚀 Overview
This repository contains the official competition code for Juggernauts Team 1 competing in the OCCRA Robotics League.
The robot program includes full teleop control, autonomous routines, mechanism management, safety features, and a custom Shuffleboard dashboard built using:
WPILib 2025 (Java)
Command-based architecture style with TimedRobot
Java 17
GradleRIO
CTRE TalonSRX + REV SparkMax motor controllers
This codebase is designed for clarity, maintainability, and reliability during competition.

🧰 System Requirements
Software
Java 17 (required)
WPILib 2025 does NOT support Java 18–21.
WPILib 2025.3.2 or later
GradleRIO (included automatically)
VS Code with WPILib Extension (recommended)
Hardware
RoboRIO (OCCRA-legal)
CTRE TalonSRX motor controllers
REV SparkMax motor controllers
Sensors (encoders, limit switches, etc.)
USB game controllers (Xbox recommended)

🤖 Robot Code Summary
This project contains a fully featured robot control system built for OCCRA competition play.
🏎️ 1. Drivetrain System
Four TalonSRX motor controllers
Right side inverted for correct forward motion
Support for multiple driving styles:
Arcade Drive
Tank Drive
Curvature Drive
Drive mode selectable via Shuffleboard chooser
Nudge control using D-Pad (precision movements)
Speed limiting:
Forward capped at 70%
Turning capped at 60%
Automatic 180° turn (A Button)
Joystick deadband and input smoothing

🏗️ 2. Mechanisms
Elevator — Spark Max (CAN ID 5)
Controlled by driver/operator triggers
Power capped at 50%
Current displayed on Shuffleboard
Manipulator / Intake — Spark Max (CAN ID 6)
Controlled by joystick Y-axis
Variable speed intake:
Intake: up to +80%
Eject: up to –50%
Status updates on Shuffleboard (INTAKE / OUTPUT / OFF)

🎮 3. Control Modes
Switchable via Shuffleboard:
Solo Mode – Driver controls everything
Co-Op Mode – Driver = drivetrain, Operator = mechanisms

🤖 4. Autonomous Routines
Selectable on Shuffleboard:
Drive Forward – Moves robot using timed forward power
Turn 180° – Spins in place for a preset duration
Autonomous runs using a WPILib timer.

🔧 5. Safety & Monitoring
Battery voltage tracking
Low-voltage warning (<10.5V)
Drive motor current monitoring
Mechanism current monitoring
TalonSRX current limiting (40A continuous, 60A peak)
Safe motor defaults on startup

📊 6. Shuffleboard Dashboard
Three fully programmed tabs:
Drive Tab
Drive mode chooser
Control mode chooser
Real-time drive outputs
D-Pad nudge indicator
Mechanism currents
Battery voltage
180° turn status
Autonomous Tab
Autonomous mode chooser
Adjustable forward-drive time
Auto status
Battery voltage
Disabled Tab
Diagnostic panel
Battery voltage
Current draw stats

🧱 7. Project Architecture
Built using WPILib’s standard robot structure:
robotInit() → hardware setup
robotPeriodic() → diagnostics & dashboard
autonomousPeriodic() → timed auto steps
teleopPeriodic() → driving & mechanism input
disabledInit() → safe shutdown behavior

📦 Vendor Libraries
Stored in /vendordeps/:
REV SparkMax
CTRE Phoenix
🙌 Acknowledgements
Thanks to:
programmers
OCCRA organizers
WPILib developers
REV Robotics & CTRE engineers
