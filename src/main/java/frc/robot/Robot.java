package frc.robot;

// --- WPILib Core Imports ---
import edu.wpi.first.wpilibj.TimedRobot; // Base class for FRC robots
import edu.wpi.first.wpilibj.XboxController; // Standard controller interface
import edu.wpi.first.wpilibj.PowerDistribution; // Access to PDH/PDP diagnostics
import edu.wpi.first.wpilibj.drive.DifferentialDrive; // Utility for two-sided drive systems
import edu.wpi.first.wpilibj.Timer; // Timing utility for autonomous and macros

// --- CTRE Imports (TalonSRX/VictorSPX) ---
import com.ctre.phoenix.motorcontrol.NeutralMode; // For setting brake/coast
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration; // For motor protection
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX; // Motor controller class

// --- REV Robotics Imports (Spark Max) ---
import com.revrobotics.spark.SparkMax; // Motor controller class
import com.revrobotics.spark.SparkLowLevel.MotorType; // Motor type (Brushed/Brushless)

// --- Dashboard/GUI Imports (Shuffleboard/SmartDashboard) ---
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser; // Widget for selecting options
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard; // Main Shuffleboard API
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab; // Individual tabs on the dashboard
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets; // Pre-made visual widgets
import edu.wpi.first.networktables.GenericEntry; // Interface for NetworkTable data entries

/**
 * This class is the primary interface for operating the FRC robot.
 * It contains all hardware definitions, configuration, and periodic control loops.
 */
public class Robot extends TimedRobot {

    // --- Hardware Definitions ---

    // Instantiate the Power Distribution Hub (PDH) - CAN ID defaults to 1
    private final PowerDistribution pdDevice = new PowerDistribution();

    // Drive train motor controllers (CAN IDs 1-4)
    private final WPI_TalonSRX leftFront = new WPI_TalonSRX(1); // Left side master
    private final WPI_TalonSRX leftRear  = new WPI_TalonSRX(2); // Left side follower
    private final WPI_TalonSRX rightFront = new WPI_TalonSRX(3); // Right side master
    private final WPI_TalonSRX rightRear = new WPI_TalonSRX(4); // Right side follower

    // Mechanism Motor CAN IDs
    private static final int CAN_ID_ELEVATOR = 5;
    private static final int CAN_ID_MANIPULATOR = 6;
    
    // Mechanism Motor Controllers (Spark Max)
    private SparkMax elevatorMotor; // Motor ID 5: Controls Up/Down
    private SparkMax manipulatorMotor; // Motor ID 6: Controls Intake/Output

    // Mechanism Constants
    private static final double ELEVATOR_MAX_SPEED = 0.5;    // 50% max output for elevator
    private static final double INTAKE_SPEED = 0.8;          // 80% speed for picking up the cube (Intake)
    private static final double OUTPUT_SPEED = -0.5;         // -50% speed for scoring the cube (Output/Eject)

    // WPILib Drive Utility: Uses the master motors (leftFront, rightFront)
    private final DifferentialDrive drive = new DifferentialDrive(leftFront, rightFront);
    
    // Timer used specifically for timing events in autonomous mode
    private final Timer autoTimer = new Timer();

    // Controllers (Driver and Operator)
    private final XboxController driver = new XboxController(0); // Driver on USB Port 0
    private final XboxController operator = new XboxController(1); // Operator on USB Port 1


    // --- Autonomous Setup ---

    // SendableChooser for selecting the autonomous routine on Shuffleboard
    private SendableChooser<String> autoChooser = new SendableChooser<>();

    // Names of the autonomous routines
    private static final String AUTO_DEFAULT = "Drive Forward";
    private static final String AUTO_TURN = "Turn 180°";
    
    // Variables to store the selected routine and its configurable drive time
    private String selectedAuto;
    private double autoDriveTime; // Configurable duration for "Drive Forward"


    // --- Teleop Configuration Constants ---

    // Scaling factors to reduce the max speed of the robot for better control
    private static final double SPEED_SCALE = 0.7; // 70% max forward/backward speed
    private static final double TURN_SCALE = 0.6; // 60% max turning speed
    private static final double DEADBAND = 0.1; // Joystick input below this value is treated as zero

    // New: Speed for the D-Pad Nudge/Bump control (25% power) - Increased from 0.15
    private static final double NUDGE_SPEED = 0.25;

    // Variables and constants for the "Turn 180" macro
    private boolean turning180 = false; // State flag: true when the macro is running
    private final Timer turnTimer = new Timer(); // Timer for the macro duration
    private static final double TURN_TIME = 1.0; // Duration of the turn in seconds
    private static final double TURN_SPEED = 0.5; // Speed during the turn


    // --- Shuffleboard/NetworkTables Definitions ---

    // General/Auto Tab Entries
    private GenericEntry autoDriveTimeEntry;
    private GenericEntry batteryVoltageEntry;
    private GenericEntry selectedAutoEntry;
    private GenericEntry autoStatusEntry;

    // Drive Tab Live Stats Entries
    private GenericEntry leftDriveCurrentEntry;
    private GenericEntry rightDriveCurrentEntry;
    private GenericEntry leftDriveOutputEntry;
    private GenericEntry rightDriveOutputEntry;
    private GenericEntry turningStatusEntry;
    
    // Mechanism Status/Current Entries
    private GenericEntry elevatorCurrentEntry;
    private GenericEntry manipulatorCurrentEntry;
    private GenericEntry manipulatorStatusEntry;

    // Controller Inputs
    private GenericEntry elevatorOutputEntry;
    private GenericEntry leftTriggerEntry;
    private GenericEntry rightTriggerEntry;

    // Warning and Mode Display Entries
    private GenericEntry batteryWarningEntryDrive;
    private GenericEntry batteryWarningEntryDisabled;
    private GenericEntry currentDriveModeEntry;
    private GenericEntry motorCurrentsDisabledEntry;

    // Control Mode Chooser
    private final SendableChooser<String> controlModeChooser = new SendableChooser<>();
    private static final String SINGLE_OPERATOR = "Solo"; // One controller for everything
    private static final String DUAL_OPERATOR = "Co-Op"; // Driver and Operator controllers used

    // Drive Mode Chooser
    private final SendableChooser<String> driveModeChooser = new SendableChooser<>();
    private static final String DRIVE_ARCADE = "Arcade";
    private static final String DRIVE_TANK = "Tank";
    private static final String DRIVE_CURVATURE = "Curvature";

    // Define Shuffleboard Tabs
    private ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");
    private ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");
    private ShuffleboardTab disabledTab = Shuffleboard.getTab("Disabled");


    /**
     * This function is run when the robot is first started (once only).
     * Used for configuring hardware and setting up dashboard widgets.
     */
    @Override
    public void robotInit() {
        // --- Motor Controller Initialization (TalonSRX Drive) ---
        // Reset all configurations to factory defaults to ensure a clean state
        leftFront.configFactoryDefault();
        leftRear.configFactoryDefault();
        rightFront.configFactoryDefault();
        rightRear.configFactoryDefault();

        // Sets the default behavior when motor output is zero: Brake for faster stopping.
        setNeutralMode(NeutralMode.Brake);

        // Configure master/follower pairs: Rear motors follow the front motors
        leftRear.follow(leftFront);
        rightRear.follow(rightFront);

        // Invert the right side motors to ensure positive input drives both sides forward
        rightFront.setInverted(true);
        rightRear.setInverted(true);

        // Configure Supply Current Limiting (CTRE Motor Protection)
        SupplyCurrentLimitConfiguration limit = new SupplyCurrentLimitConfiguration(true, 40, 60, 0.1);
        leftFront.configSupplyCurrentLimit(limit);
        leftRear.configSupplyCurrentLimit(limit);
        rightFront.configSupplyCurrentLimit(limit);
        rightRear.configSupplyCurrentLimit(limit);

        // --- Mechanism Motor Initialization (SparkMax) ---
        // Initialize SparkMax for Elevator (ID 5) and Manipulator (ID 6)
        elevatorMotor = new SparkMax(CAN_ID_ELEVATOR, MotorType.kBrushless);
        manipulatorMotor = new SparkMax(CAN_ID_MANIPULATOR, MotorType.kBrushless);

        // --- Shuffleboard and Chooser Initialization (Same as before) ---

        // Setup Drive Mode Chooser
        driveModeChooser.setDefaultOption(DRIVE_ARCADE, DRIVE_ARCADE);
        driveModeChooser.addOption(DRIVE_TANK, DRIVE_TANK);
        driveModeChooser.addOption(DRIVE_CURVATURE, DRIVE_CURVATURE);

        // Setup Control Mode Chooser
        controlModeChooser.setDefaultOption(SINGLE_OPERATOR, SINGLE_OPERATOR);
        controlModeChooser.addOption(DUAL_OPERATOR, DUAL_OPERATOR);

        // =======================================================================
        // --- Shuffleboard Tab Setup: Drive Tab (Optimized for 10x4 Grid) ---
        // =======================================================================

        // Row 0: Modes and Status (Total Width: 3 + 3 + 3 + 1 = 10. Perfect.)
        driveTab.add("Control Mode", controlModeChooser)
                .withWidget(BuiltInWidgets.kComboBoxChooser)
                .withPosition(0, 0).withSize(3, 1); // [0, 0, 3, 1]

        driveTab.add("Drive Mode", driveModeChooser)
                .withWidget(BuiltInWidgets.kComboBoxChooser)
                .withPosition(3, 0).withSize(3, 1); // [3, 0, 3, 1]

        currentDriveModeEntry = driveTab.add("Current Drive Mode", DRIVE_ARCADE)
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(6, 0).withSize(3, 1) // [6, 0, 3, 1]
                .getEntry();

        turningStatusEntry = driveTab.add("180 Turn Active", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(9, 0).withSize(1, 1) // [9, 0, 1, 1]
                .getEntry();
        
        // Row 1: Drive Train Live Stats (Total Width: 3 + 3 + 2 + 2 = 10. Perfect.)
        leftDriveCurrentEntry = driveTab.add("Left Current (A)", 0.0)
                .withWidget(BuiltInWidgets.kVoltageView)
                .withPosition(0, 1).withSize(3, 1) // [0, 1, 3, 1]
                .getEntry();

        rightDriveCurrentEntry = driveTab.add("Right Current (A)", 0.0)
                .withWidget(BuiltInWidgets.kVoltageView)
                .withPosition(3, 1).withSize(3, 1) // [3, 1, 3, 1]
                .getEntry();

        leftDriveOutputEntry = driveTab.add("Left Output (%)", 0.0)
                .withWidget(BuiltInWidgets.kDial)
                .withPosition(6, 1).withSize(2, 1) // [6, 1, 2, 1]
                .getEntry();

        rightDriveOutputEntry = driveTab.add("Right Output (%)", 0.0)
                .withWidget(BuiltInWidgets.kDial)
                .withPosition(8, 1).withSize(2, 1) // [8, 1, 2, 1]
                .getEntry();

        // Row 2: Mechanism Stats (Total Width: 2 + 2 + 3 + 3 = 10. Perfect.)
        elevatorOutputEntry = driveTab.add("Elevator Output (%)", 0)
                .withWidget(BuiltInWidgets.kDial)
                .withPosition(0, 2).withSize(2, 1) // [0, 2, 2, 1]
                .getEntry();

        elevatorCurrentEntry = driveTab.add("Elevator Current (A)", 0.0)
                .withWidget(BuiltInWidgets.kNumberBar)
                .withPosition(2, 2).withSize(2, 1) // [2, 2, 2, 1]
                .getEntry();

        manipulatorCurrentEntry = driveTab.add("Manipulator Current (A)", 0.0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withPosition(4, 2).withSize(3, 1) // [4, 2, 3, 1]
            .getEntry();
        
        manipulatorStatusEntry = driveTab.add("Manipulator Status", "OFF")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(7, 2).withSize(3, 1) // [7, 2, 3, 1]
            .getEntry();

        // Row 3: Trigger Inputs and Battery Warning (Total Width: 3 + 3 + 4 = 10. Perfect.)
        leftTriggerEntry = driveTab.add("Left Trigger (Elevator Down)", 0)
                .withWidget(BuiltInWidgets.kNumberBar)
                .withPosition(0, 3).withSize(3, 1) // [0, 3, 3, 1]
                .getEntry();

        rightTriggerEntry = driveTab.add("Right Trigger (Elevator Up)", 0)
                .withWidget(BuiltInWidgets.kNumberBar)
                .withPosition(3, 3).withSize(3, 1) // [3, 3, 3, 1]
                .getEntry();
                
        batteryWarningEntryDrive = driveTab.add("Battery Status", "OK")
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(6, 3).withSize(4, 1) // [6, 3, 4, 1]
                .getEntry();


        // =======================================================================
        // --- Shuffleboard Tab Setup: Autonomous Tab (Optimized for 10x4 Grid) ---
        // =======================================================================

        // Row 0: Configuration and Health (Total Width: 3 + 2 + 3 + 2 = 10. Perfect.)
        autoChooser.setDefaultOption("Drive Forward", AUTO_DEFAULT);
        autoChooser.addOption("Turn 180°", AUTO_TURN);

        autoTab.add("1. Select Autonomous Mode", autoChooser)
           .withPosition(0, 0).withSize(3, 1); // [0, 0, 3, 1]

        autoDriveTimeEntry = autoTab.add("2. Auto Drive Time (s)", 2.0)
            .withPosition(3, 0).withSize(2, 1) // [3, 0, 2, 1]
            .getEntry();

        batteryVoltageEntry = autoTab.add("Battery Voltage (V)", 12.5)
            .withWidget(BuiltInWidgets.kVoltageView)
            .withPosition(5, 0).withSize(3, 1) // [5, 0, 3, 1]
            .getEntry();

        selectedAutoEntry = autoTab.add("Selected Auto", AUTO_DEFAULT)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(8, 0).withSize(2, 1) // [8, 0, 2, 1]
            .getEntry();
        
        // Row 1: Execution Status (Total Width: 10. Full width for clarity.)
        autoStatusEntry = autoTab.add("Auto Status", "Ready for Init")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 1).withSize(10, 1) // [0, 1, 10, 1]
            .getEntry();

        // =======================================================================
        // --- Shuffleboard Tab Setup: Disabled Tab (Optimized for 10x4 Grid) ---
        // =======================================================================

        // Row 0: Primary Warnings
        batteryWarningEntryDisabled = disabledTab.add("Battery Status", "OK")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 0).withSize(10, 1) // [0, 0, 10, 1]
            .getEntry();
            
        // Row 1: Diagnostics
        motorCurrentsDisabledEntry = disabledTab.add("Disabled Current Check", "Checking...")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 1).withSize(10, 1) // [0, 1, 10, 1]
            .getEntry();
    }
    
    /**
     * This function runs at a constant rate (e.g., 50Hz) regardless of the robot's mode.
     * Ideal for diagnostic updates and safety checks.
     */
    @Override
    public void robotPeriodic() {
        // --- Universal Diagnostics Updates ---
        
        // Read the actual bus voltage from the PDH/PDP
        double voltage = pdDevice.getVoltage();
        batteryVoltageEntry.setDouble(voltage); // Push value to NetworkTables

        // Safety Check: Low battery warning logic
        if (voltage < 10.5) { 
            String warning = "!!! LOW BATTERY: " + String.format("%.2f", voltage) + " V !!!";
            batteryWarningEntryDrive.setString(warning);
            batteryWarningEntryDisabled.setString(warning);
        } else {
            batteryWarningEntryDrive.setString("Battery OK: " + String.format("%.2f", voltage) + " V");
            batteryWarningEntryDisabled.setString("Battery OK: " + String.format("%.2f", voltage) + " V");
        }

        // --- Drive Train Live Stats Update ---
        
        // Update motor output percentages (0% to 100%)
        leftDriveOutputEntry.setDouble(leftFront.getMotorOutputPercent() * 100.0);
        rightDriveOutputEntry.setDouble(rightFront.getMotorOutputPercent() * 100.0);

        // Calculate and update combined current draw for each side of the drive train
        double leftCurrent = leftFront.getSupplyCurrent() + leftRear.getSupplyCurrent();
        double rightCurrent = rightFront.getSupplyCurrent() + rightRear.getSupplyCurrent();
        leftDriveCurrentEntry.setDouble(leftCurrent);
        rightDriveCurrentEntry.setDouble(rightCurrent);

        // Update mechanism current draw (Individual motors)
        double elevatorCurrent = elevatorMotor.getOutputCurrent();
        elevatorCurrentEntry.setDouble(elevatorCurrent);

        double manipulatorCurrent = manipulatorMotor.getOutputCurrent();
        manipulatorCurrentEntry.setDouble(manipulatorCurrent);
    }

    /**
     * This function is called continuously during teleoperated mode.
     * Contains the main driver and operator control logic.
     */
    @Override
    public void teleopPeriodic() {
        double speedScale = SPEED_SCALE;
        double turnScale = TURN_SCALE;
        
        turningStatusEntry.setBoolean(turning180);

        // --- 1. D-Pad Nudge/Bump Control Logic (Highest Priority) ---
        // Checks the Driver's D-Pad (POV) for precise, slow-speed movement.
        int povAngle = driver.getPOV();

        if (povAngle != -1) {
            // If the D-Pad is pressed, use the NUDGE speed and bypass joystick control
            switch (povAngle) {
                case 0: // Up: Forward Nudge
                    drive.arcadeDrive(NUDGE_SPEED, 0.0);
                    break;
                case 180: // Down: Reverse Nudge
                    drive.arcadeDrive(-NUDGE_SPEED, 0.0);
                    break;
                case 270: // Left: Turn Left Nudge
                    drive.arcadeDrive(0.0, -NUDGE_SPEED);
                    break;
                case 90: // Right: Turn Right Nudge
                    drive.arcadeDrive(0.0, NUDGE_SPEED);
                    break;
                case 45: // Up-Right: Forward + Turn Right Nudge
                    drive.arcadeDrive(NUDGE_SPEED, NUDGE_SPEED);
                    break;
                case 135: // Down-Right: Reverse + Turn Right Nudge
                    drive.arcadeDrive(-NUDGE_SPEED, NUDGE_SPEED);
                    break;
                case 225: // Down-Left: Reverse + Turn Left Nudge
                    drive.arcadeDrive(-NUDGE_SPEED, -NUDGE_SPEED);
                    break;
                case 315: // Up-Left: Forward + Turn Left Nudge
                    drive.arcadeDrive(NUDGE_SPEED, -NUDGE_SPEED);
                    break;
                default:
                    drive.stopMotor();
                    break;
            }
            // Crucial: Stop processing drive inputs from analog sticks if D-Pad is active
            return; 
        }

        // --- 180 Turn Macro Logic --- (Driver only)
        if (driver.getAButtonPressed() && !turning180) {
            turning180 = true;
            turnTimer.reset();
            turnTimer.start();
        }

        if (turning180) {
            if (turnTimer.get() < TURN_TIME) {
                drive.tankDrive(TURN_SPEED, -TURN_SPEED);
                return; // Prevents subsequent drive code from overriding the turn
            } else {
                drive.stopMotor();
                turning180 = false;
                turnTimer.stop();
            }
        }

        // --- Control Mode Selection and Input Reading ---
        String controlMode = controlModeChooser.getSelected();
        if (controlMode == null) controlMode = SINGLE_OPERATOR; 

        // Determine which controller handles the mechanism inputs (Elevator, Manipulator)
        XboxController mechController = (DUAL_OPERATOR.equals(controlMode)) ? operator : driver;

        // Initialize all movement variables
        double forward = 0;
        double turn = 0;
        double left = 0;
        double right = 0;
        double elevatorSpeed = 0;

        // --- Drive and Mechanism Input Mapping ---
        if (SINGLE_OPERATOR.equals(controlMode)) {
            // Drive inputs
            forward = -driver.getLeftY(); // LEFT STICK Y-AXIS for Speed (Arcade/Curvature)
            turn = driver.getLeftX();    // LEFT STICK X-AXIS for Turning (Arcade/Curvature)
            left = -driver.getLeftY();   // LEFT STICK Y-AXIS for Tank
            right = -driver.getRightY(); // RIGHT STICK Y-AXIS for Tank
            
            // Mechanism inputs (from Driver)
            double rightTrigger = driver.getRightTriggerAxis();
            double leftTrigger = driver.getLeftTriggerAxis();
            elevatorSpeed = (leftTrigger - rightTrigger) * ELEVATOR_MAX_SPEED;

            leftTriggerEntry.setDouble(leftTrigger);
            rightTriggerEntry.setDouble(rightTrigger);

        } else if (DUAL_OPERATOR.equals(controlMode)) {
            // Drive inputs (from Driver)
            forward = -driver.getLeftY(); // LEFT STICK Y-AXIS for Speed (Arcade/Curvature)
            turn = driver.getLeftX();    // LEFT STICK X-AXIS for Turning (Arcade/Curvature)
            left = -driver.getLeftY();   // LEFT STICK Y-AXIS for Tank
            right = -driver.getRightY(); // RIGHT STICK Y-AXIS for Tank
             
            // Mechanism inputs (from Operator)
            double rightTrigger = operator.getRightTriggerAxis();
            double leftTrigger = operator.getLeftTriggerAxis();
            elevatorSpeed = (leftTrigger - rightTrigger) * ELEVATOR_MAX_SPEED;

            leftTriggerEntry.setDouble(leftTrigger);
            rightTriggerEntry.setDouble(rightTrigger);
        }

        // ---------------------------------------------------------------------
        // --- 2. Variable Speed Manipulator Control Logic ---
        // Uses mechController's Right Y-Axis for proportional speed control.
        // ---------------------------------------------------------------------
        double manipulatorInput = mechController.getRightY(); // -1.0 (Forward) to 1.0 (Backward)
        double manipulatorOutput = 0.0;
        
        if (Math.abs(manipulatorInput) > DEADBAND) {
            if (manipulatorInput < 0) {
                // Stick Forward (Intake): Output is positive, scaled by INTAKE_SPEED
                // The input is negative, so we multiply by -1 to get magnitude (0 to 1)
                manipulatorOutput = -manipulatorInput * INTAKE_SPEED; 
                manipulatorStatusEntry.setString("INTAKE (Prop: " + String.format("%.2f", manipulatorOutput) + ")");
            } else {
                // Stick Backward (Output): Output is negative, scaled by OUTPUT_SPEED (which is negative)
                // The input is positive, so we scale by magnitude * OUTPUT_SPEED
                manipulatorOutput = manipulatorInput * OUTPUT_SPEED;
                manipulatorStatusEntry.setString("OUTPUT (Prop: " + String.format("%.2f", manipulatorOutput) + ")");
            }
        } else {
            manipulatorOutput = 0.0;
            manipulatorStatusEntry.setString("OFF");
        }

        manipulatorMotor.set(manipulatorOutput);


        // --- Drive Input Post-Processing ---

        // Apply Deadband
        forward = applyDeadband(forward, DEADBAND);
        turn = applyDeadband(turn, DEADBAND);
        left = applyDeadband(left, DEADBAND);
        right = applyDeadband(right, DEADBAND);
        
        // Square the turn input for finer control
        turn = Math.copySign(turn * turn, turn);

        // --- Drive Mode Execution ---
        String mode = driveModeChooser.getSelected();
        if (mode == null) mode = DRIVE_ARCADE;
        currentDriveModeEntry.setString(mode);

        switch (mode) {
            case DRIVE_TANK:
                // TANK drive uses the Left and Right Y axes (separate sticks)
                drive.tankDrive(left * speedScale, right * speedScale);
                break;
            case DRIVE_CURVATURE:
                // CURVATURE drive uses the Left Stick (Y for speed, X for turn)
                drive.curvatureDrive(forward * speedScale, turn * turnScale, true);
                break;
            case DRIVE_ARCADE:
            default:
                // ARCADE drive uses the Left Stick (Y for speed, X for turn)
                drive.arcadeDrive(forward * speedScale, turn * turnScale);
                break;
        }

        // --- Elevator Control Execution ---
        
        // Apply deadband
        if (Math.abs(elevatorSpeed) < 0.05) elevatorSpeed = 0;
        
        // Set motor speed (ONLY using the dedicated elevator motor)
        elevatorMotor.set(elevatorSpeed);
        
        elevatorOutputEntry.setDouble(elevatorSpeed * 100.0 / ELEVATOR_MAX_SPEED); 
    }

    /**
     * Helper method to apply a deadband to a joystick input value.
     */
    private double applyDeadband(double value, double deadband) {
        return Math.abs(value) < deadband ? 0.0 : value;
    }

    /**
     * Utility function to set the neutral mode for all four drive motors.
     */
    private void setNeutralMode(NeutralMode mode) {
        leftFront.setNeutralMode(mode);
        leftRear.setNeutralMode(mode);
        rightFront.setNeutralMode(mode);
        rightRear.setNeutralMode(mode);
    }

    /**
     * Runs once when the robot enters disabled mode.
     */
    @Override
    public void disabledInit() {
        setNeutralMode(NeutralMode.Coast);
        drive.stopMotor();
        
        // Disabled Current Diagnostic Check
        double totalCurrent = pdDevice.getTotalCurrent();
        if (totalCurrent > 5.0) { 
            motorCurrentsDisabledEntry.setString("WARNING: High Disabled Current (" + String.format("%.1f", totalCurrent) + " A)! Check wiring.");
        } else {
            motorCurrentsDisabledEntry.setString("Disabled Current Check: OK (Total: " + String.format("%.1f", totalCurrent) + " A)");
        }
    }
    
    /**
     * Runs once when the robot enters autonomous mode.
     */
    @Override
    public void autonomousInit() {
        selectedAuto = autoChooser.getSelected();
        selectedAutoEntry.setString(selectedAuto);
        autoDriveTime = autoDriveTimeEntry.getDouble(2.0);
        autoTimer.reset();
        autoTimer.start();
        autoStatusEntry.setString("Starting Autonomous: " + selectedAuto);
        setNeutralMode(NeutralMode.Brake);
        drive.stopMotor();
    }
    
    /**
     * Runs continuously during autonomous mode.
     */
    @Override
    public void autonomousPeriodic() {
        double remainingTime = autoDriveTime - autoTimer.get();
        String status = "Time Remaining: " + String.format("%.2f", remainingTime) + "s";
        
        // Autonomous Execution Logic
        switch (selectedAuto) {
            case AUTO_TURN:
                if (autoTimer.get() < TURN_TIME) {
                    drive.tankDrive(TURN_SPEED, -TURN_SPEED);
                    autoStatusEntry.setString("Executing Turn 180°. " + status);
                } else {
                    drive.stopMotor();
                    autoStatusEntry.setString("Turn Complete. Motor Stopped.");
                }
                break;

            case AUTO_DEFAULT:
            default:
                if (autoTimer.get() < autoDriveTime) {
                    drive.arcadeDrive(0.5, 0.0);
                    autoStatusEntry.setString("Driving Forward. " + status);
                } else {
                    drive.stopMotor();
                    autoStatusEntry.setString("Drive Complete. Motor Stopped.");
                }
                break;
            }
    }
}