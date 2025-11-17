package frc.robot;

// --- CTRE Imports (TalonSRX/VictorSPX) ---
import com.ctre.phoenix.motorcontrol.NeutralMode; // For setting motor behavior when output is zero (Brake or Coast)
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration; // Configuration object for setting motor current limits
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX; // WPI class for TalonSRX motor controller (used for drive)
import com.revrobotics.spark.SparkLowLevel.MotorType; // Enum to specify motor type (kBrushless or kBrushed)
// --- REV Robotics Imports (Spark Max) ---
import com.revrobotics.spark.SparkMax; // Motor controller class for REV Spark Max (used for mechanisms)

import edu.wpi.first.networktables.GenericEntry; // Interface for creating and updating NetworkTable/Shuffleboard data entries
import edu.wpi.first.wpilibj.PowerDistribution; // Access to Power Distribution Hub (PDH) diagnostics (voltage, current)
// --- WPILib Core Imports ---
import edu.wpi.first.wpilibj.TimedRobot; // Base class for FRC robots, provides fixed-rate periodic methods
import edu.wpi.first.wpilibj.Timer; // Utility for timing events (used in autonomous and the 180-degree macro)
import edu.wpi.first.wpilibj.XboxController; // Standard interface for reading Xbox controller inputs
import edu.wpi.first.wpilibj.drive.DifferentialDrive; // Utility class to handle two-sided drive systems (Arcade, Tank, Curvature)
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets; // Pre-defined visual widgets for the Shuffleboard dashboard
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard; // Main API for creating and managing the Shuffleboard dashboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab; // Represents an individual tab on the Shuffleboard dashboard
// --- Dashboard/GUI Imports (Shuffleboard/SmartDashboard) ---
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser; // Widget for selecting options (Autonomous modes, Drive modes)

/**
 * This class is the primary interface for operating the FRC robot.
 * It extends TimedRobot, providing the main control loop structure.
 * It contains all hardware definitions, configuration, and periodic control logic.
 */
public class Robot extends TimedRobot {

    // --- Hardware Definitions ---

    // Instantiate the Power Distribution Hub (PDH) - CAN ID defaults to 1
    private final PowerDistribution pdDevice = new PowerDistribution();

    // Drive train motor controllers (TalonSRX, CAN IDs 1-4)
    private final WPI_TalonSRX leftFront = new WPI_TalonSRX(1); // Left side master
    private final WPI_TalonSRX leftRear  = new WPI_TalonSRX(2); // Left side follower
    private final WPI_TalonSRX rightFront = new WPI_TalonSRX(3); // Right side master
    private final WPI_TalonSRX rightRear = new WPI_TalonSRX(4); // Right side follower

    // Mechanism CAN IDs (Defined as constants for clarity)
    private static final int CAN_ID_ELEVATOR = 5;
    private static final int CAN_ID_MANIPULATOR = 6;
    
    // Mechanism Motor Controllers (Spark Max)
    // Note: These are initialized in robotInit() to correctly specify MotorType
    private SparkMax elevatorMotor; // Motor ID 5: Controls Up/Down movement
    private SparkMax manipulatorMotor; // Motor ID 6: Controls Intake/Output action

    // Mechanism Tuning Constants
    private static final double ELEVATOR_MAX_SPEED = 0.5;  // Max output for elevator (50%) to prevent excessive strain
    private static final double INTAKE_SPEED = 0.8;        // Max speed for picking up the game piece (80%)
    private static final double OUTPUT_SPEED = -0.5;       // Max speed for scoring/ejecting the game piece (-50%)

    // WPILib Drive Utility: Handles the math for drive modes (Arcade, Tank, Curvature)
    // It uses only the master motors (leftFront, rightFront)
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
    private double autoDriveTime; // Configurable duration for the "Drive Forward" routine


    // --- Teleop Configuration Constants ---

    // Scaling factors to reduce the max speed of the robot for better control
    private static final double SPEED_SCALE = 0.7; // 70% max forward/backward speed
    private static final double TURN_SCALE = 0.6; // 60% max turning speed
    private static final double DEADBAND = 0.1; // Joystick input below this absolute value is treated as zero

    // Speed for the D-Pad Nudge/Bump control (40% power)
    private static final double NUDGE_SPEED = 0.40;

    // Variables and constants for the "Turn 180" macro (A button)
    private boolean turning180 = false; // State flag: true when the macro is running
    private final Timer turnTimer = new Timer(); // Timer for the macro duration
    private static final double TURN_TIME = 1.69; // Calibrated duration of the turn in seconds (1.69s for 180 degrees)
    private static final double TURN_SPEED = 0.5; // Speed during the turn (50% power)


    // --- Shuffleboard/NetworkTables Definitions (GenericEntry allows reading/writing) ---

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

    // Controller Inputs (for debugging)
    private GenericEntry elevatorOutputEntry;
    private GenericEntry leftTriggerEntry;
    private GenericEntry rightTriggerEntry;

    // Warning and Mode Display Entries
    private GenericEntry batteryWarningEntryDrive;
    private GenericEntry batteryWarningEntryDisabled;
    private GenericEntry currentDriveModeEntry;
    private GenericEntry motorCurrentsDisabledEntry;

    // Control Mode Chooser (Dropdown for selecting Solo or Co-Op mode)
    private final SendableChooser<String> controlModeChooser = new SendableChooser<>();
    private static final String SINGLE_OPERATOR = "Solo"; // One controller for everything
    private static final String DUAL_OPERATOR = "Co-Op"; // Driver and Operator controllers used

    // Drive Mode Chooser (Dropdown for selecting Arcade, Tank, or Curvature mode)
    private final SendableChooser<String> driveModeChooser = new SendableChooser<>();
    private static final String DRIVE_ARCADE = "Arcade";
    private static final String DRIVE_TANK = "Tank";
    private static final String DRIVE_CURVATURE = "Curvature";

    // Define Shuffleboard Tabs for organization
    private ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");      //Tab For Teleop Period
    private ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");  //Tab For Autonomous Period
    private ShuffleboardTab disabledTab = Shuffleboard.getTab("Disabled");//Tab For Disabled Period


    /**
     * This function is run when the robot is first started (once only).
     * Used for configuring hardware and setting up dashboard widgets.
     */
    @Override
    public void robotInit() {
        // --- Motor Controller Initialization (TalonSRX Drive) ---
        // Reset all configurations to factory defaults to ensure a clean slate
        leftFront.configFactoryDefault();
        leftRear.configFactoryDefault();
        rightFront.configFactoryDefault();
        rightRear.configFactoryDefault();

        // Configure Open Loop Ramp Rate (Ramping from 0% to 100% output over 0.25 seconds)
        // This makes the robot accelerate smoothly, preventing wheel spin and unnecessary current draw.
        leftFront.configOpenloopRamp(0.25); 
        leftRear.configOpenloopRamp(0.25);
        rightFront.configOpenloopRamp(0.25);
        rightRear.configOpenloopRamp(0.25);
        
        // Sets the default motor behavior when output is zero: Brake for faster stopping.
        setNeutralMode(NeutralMode.Brake);

        // Configure master/follower pairs: Rear motors mirror the command of the front motors
        leftRear.follow(leftFront);
        rightRear.follow(rightFront);

        // Invert the right side motors to ensure positive input drives both sides forward
        rightFront.setInverted(true);
        rightRear.setInverted(true);

        // Configure Supply Current Limiting (CTRE Motor Protection)
        // Enable: true, Continuous: 40A, Peak: 60A, Peak Time: 0.1s
        SupplyCurrentLimitConfiguration limit = new SupplyCurrentLimitConfiguration(true, 40, 60, 0.1);
        leftFront.configSupplyCurrentLimit(limit);
        leftRear.configSupplyCurrentLimit(limit);
        rightFront.configSupplyCurrentLimit(limit);
        rightRear.configSupplyCurrentLimit(limit);

        // --- Mechanism Motor Initialization (SparkMax) ---
        // Initialize SparkMax for Elevator (ID 5) and Manipulator (ID 6) as kBrushless (NEO motors)
        elevatorMotor = new SparkMax(CAN_ID_ELEVATOR, MotorType.kBrushless);
        manipulatorMotor = new SparkMax(CAN_ID_MANIPULATOR, MotorType.kBrushless);
        
        // --- Shuffleboard and Chooser Initialization ---

        // Setup Drive Mode Chooser (Arcade is the default)
        driveModeChooser.setDefaultOption(DRIVE_ARCADE, DRIVE_ARCADE);
        driveModeChooser.addOption(DRIVE_TANK, DRIVE_TANK);
        driveModeChooser.addOption(DRIVE_CURVATURE, DRIVE_CURVATURE);

        // Setup Control Mode Chooser (Solo/Single Operator is the default)
        controlModeChooser.setDefaultOption(SINGLE_OPERATOR, SINGLE_OPERATOR);
        controlModeChooser.addOption(DUAL_OPERATOR, DUAL_OPERATOR);

       // =======================================================================
        // --- Shuffleboard Tab Setup: Drive Tab (Configured for a 10-unit width) ---
        // =======================================================================

        // Row 0: Modes and Status
        // Control Mode Selector: Allows the user to select between 'Solo' (1 driver) or 'Co-Op' (Driver/Operator) control modes.
        driveTab.add("Control Mode", controlModeChooser)
                .withWidget(BuiltInWidgets.kComboBoxChooser) // Widget: Dropdown box for selecting one option from a list.
                .withPosition(0, 0).withSize(3, 1); 
        // Drive Mode Selector: Allows the user to select between Arcade, Tank, or Curvature drive styles.
        driveTab.add("Drive Mode", driveModeChooser)
                .withWidget(BuiltInWidgets.kComboBoxChooser) // Widget: Dropdown box for selecting one option from a list.
                .withPosition(3, 0).withSize(3, 1); 

        // Current Drive Mode Display: Shows the drive style currently being executed by the robot (e.g., "Arcade").
        currentDriveModeEntry = driveTab.add("Current Drive Mode", DRIVE_ARCADE)
                .withWidget(BuiltInWidgets.kTextView) // Widget: Displays a text/string value.
                .withPosition(6, 0).withSize(3, 1) 
                .getEntry();

        // 180 Turn Status: Displays a true/false (boolean) state indicating if the 180-degree turn macro is running.
        turningStatusEntry = driveTab.add("180 Turn Active", false)
                .withWidget(BuiltInWidgets.kBooleanBox) // Widget: A colored box that lights up (true) or turns off (false).
                .withPosition(9, 0).withSize(1, 1) 
                .getEntry();
        
        // Row 1: Drive Train Live Stats (Current and Output %)
        // Left Drive Current: Displays the total electrical current draw (A) of the left drivetrain motors.
        leftDriveCurrentEntry = driveTab.add("Left Current (A)", 0.0)
                .withWidget(BuiltInWidgets.kVoltageView) // Widget: Visual gauge typically used for voltage, but here repurposed for current with a warning range.
                .withPosition(0, 1).withSize(3, 1) 
                .getEntry();

        // Right Drive Current: Displays the total electrical current draw (A) of the right drivetrain motors.
        rightDriveCurrentEntry = driveTab.add("Right Current (A)", 0.0)
                .withWidget(BuiltInWidgets.kVoltageView) // Widget: Visual gauge for current draw.
                .withPosition(3, 1).withSize(3, 1) 
                .getEntry();

        // Left Drive Output: Displays the power percentage (%) commanded to the left drivetrain master motor.
        leftDriveOutputEntry = driveTab.add("Left Output (%)", 0.0)
                .withWidget(BuiltInWidgets.kDial) // Widget: A rotating gauge for showing proportional output (-100% to 100%).
                .withPosition(6, 1).withSize(2, 1) 
                .getEntry();

        // Right Drive Output: Displays the power percentage (%) commanded to the right drivetrain master motor.
        rightDriveOutputEntry = driveTab.add("Right Output (%)", 0.0)
                .withWidget(BuiltInWidgets.kDial) // Widget: A rotating gauge for showing proportional output.
                .withPosition(8, 1).withSize(2, 1) 
                .getEntry();

        // Row 2: Mechanism Stats (Output, Current, and Status)
        // Elevator Output: Displays the power percentage (%) commanded to the elevator motor.
        elevatorOutputEntry = driveTab.add("Elevator Output (%)", 0)
                .withWidget(BuiltInWidgets.kDial) // Widget: A rotating gauge for showing proportional output.
                .withPosition(0, 2).withSize(2, 1) 
                .getEntry();

        // Elevator Current: Displays the electrical current draw (A) of the elevator motor.
        elevatorCurrentEntry = driveTab.add("Elevator Current (A)", 0.0)
                .withWidget(BuiltInWidgets.kNumberBar) // Widget: A horizontal bar that fills to visualize a numerical value relative to a max.
                .withPosition(2, 2).withSize(2, 1) 
                .getEntry();

        // Manipulator Current: Displays the electrical current draw (A) of the manipulator/intake motor.
        manipulatorCurrentEntry = driveTab.add("Manipulator Current (A)", 0.0)
            .withWidget(BuiltInWidgets.kNumberBar) // Widget: A horizontal bar that fills to visualize a numerical value relative to a max.
            .withPosition(4, 2).withSize(3, 1) 
            .getEntry();
        
        // Manipulator Status: Displays the current operational mode of the manipulator mechanism (e.g., "INTAKE," "OUTPUT," or "OFF").
        manipulatorStatusEntry = driveTab.add("Manipulator Status", "OFF")
            .withWidget(BuiltInWidgets.kTextView) // Widget: Displays a text/string status.
            .withPosition(7, 2).withSize(3, 1) 
            .getEntry();

        // Row 3: Trigger Inputs and Battery Warning 
        // Left Trigger Input: Displays the driver's input value (0.0 to 1.0) from the left trigger. Used for elevator down.
        leftTriggerEntry = driveTab.add("Left Trigger (Elevator Down)", 0)
                .withWidget(BuiltInWidgets.kNumberBar) // Widget: A horizontal bar to visualize trigger pull depth.
                .withPosition(0, 3).withSize(3, 1) 
                .getEntry();

        // Right Trigger Input: Displays the driver's input value (0.0 to 1.0) from the right trigger. Used for elevator up.
        rightTriggerEntry = driveTab.add("Right Trigger (Elevator Up)", 0)
                .withWidget(BuiltInWidgets.kNumberBar) // Widget: A horizontal bar to visualize trigger pull depth.
                .withPosition(3, 3).withSize(3, 1) 
                .getEntry();
                
        // Battery Status Display: Shows the current battery voltage status with a warning message if critically low.
        batteryWarningEntryDrive = driveTab.add("Battery Status", "OK")
                .withWidget(BuiltInWidgets.kTextView) // Widget: Displays a text/string status (often colored).
                .withPosition(6, 3).withSize(4, 1) 
                .getEntry();


        // =======================================================================
        // --- Shuffleboard Tab Setup: Autonomous Tab ---
        // =======================================================================

        // Row 0: Configuration and Health
        // Autonomous Mode Selector: Defines the options for the Auto Chooser widget.
        autoChooser.setDefaultOption("Drive Forward", AUTO_DEFAULT);
        autoChooser.addOption("Turn 180°", AUTO_TURN);

        // Auto Chooser Widget: Allows selecting the autonomous routine before the match.
        autoTab.add("1. Select Autonomous Mode", autoChooser)
           .withPosition(0, 0).withSize(3, 1); 

        // Auto Drive Time Setting: Allows setting the duration (seconds) of the default auto routine.
        autoDriveTimeEntry = autoTab.add("2. Auto Drive Time (s)", 2.0)
            .withPosition(3, 0).withSize(2, 1) 
            .getEntry(); // Widget defaults to a simple Number field.

        // Battery Voltage Display: Shows the current battery voltage (V) on a visual gauge.
        batteryVoltageEntry = autoTab.add("Battery Voltage (V)", 12.5)
            .withWidget(BuiltInWidgets.kVoltageView) // Widget: Visual gauge for voltage.
            .withPosition(5, 0).withSize(3, 1) 
            .getEntry();

        // Selected Auto Confirmation: Displays the text of the currently chosen autonomous routine.
        selectedAutoEntry = autoTab.add("Selected Auto", AUTO_DEFAULT)
            .withWidget(BuiltInWidgets.kTextView) // Widget: Displays a text/string confirmation.
            .withPosition(8, 0).withSize(2, 1) 
            .getEntry();
        
        // Row 1: Execution Status (Full width for visibility)
        // Auto Status Logger: Displays verbose status messages during the autonomous period (e.g., "Driving," "Complete," "Time Remaining").
        autoStatusEntry = autoTab.add("Auto Status", "Ready for Init")
            .withWidget(BuiltInWidgets.kTextView) // Widget: Displays a text/string log/status.
            .withPosition(0, 1).withSize(10, 1) 
            .getEntry();

        // =======================================================================
        // --- Shuffleboard Tab Setup: Disabled Tab (Diagnostics) ---
        // =======================================================================

        // Row 0: Primary Warnings
        // Disabled Battery Status: Displays the current battery status while the robot is disabled (often mirrored from the Drive Tab).
        batteryWarningEntryDisabled = disabledTab.add("Battery Status", "OK")
            .withWidget(BuiltInWidgets.kTextView) // Widget: Displays a text/string warning.
            .withPosition(0, 0).withSize(10, 1) 
            .getEntry();
            
        // Row 1: Diagnostics
        // Disabled Current Check: Logs the result of a test run in disabledInit() to check for unusually high current draw while motors are commanded off.
        motorCurrentsDisabledEntry = disabledTab.add("Disabled Current Check", "Checking...")
            .withWidget(BuiltInWidgets.kTextView) // Widget: Displays a text/string diagnostic result.
            .withPosition(0, 1).withSize(10, 1) 
            .getEntry();
    }
    
    /**
     * This function runs at a constant rate (e.g., 50Hz) regardless of the robot's mode.
     * Ideal for diagnostic updates and universal safety checks.
     */
    @Override
    public void robotPeriodic() {
        // --- Universal Diagnostics Updates ---
        
        // Read the actual bus voltage from the PDH/PDP
        double voltage = pdDevice.getVoltage();
        batteryVoltageEntry.setDouble(voltage); // Push value to NetworkTables (Autonomous Tab)

        // Safety Check: Low battery warning logic (Threshold 10.5V)
        if (voltage < 10.5) { 
            String warning = "!!! LOW BATTERY: " + String.format("%.2f", voltage) + " V !!!";
            batteryWarningEntryDrive.setString(warning);
            batteryWarningEntryDisabled.setString(warning);
        } else {
            batteryWarningEntryDrive.setString("Battery OK: " + String.format("%.2f", voltage) + " V");
            batteryWarningEntryDisabled.setString("Battery OK: " + String.format("%.2f", voltage) + " V");
        }

        // --- Drive Train Live Stats Update ---
        
        // Update motor output percentages (0% to 100%) from the master motors
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
     * Contains the main driver and operator control logic hierarchy.
     */
    @Override
    public void teleopPeriodic() {
        // Use local variables for speed scales
        double speedScale = SPEED_SCALE;
        double turnScale = TURN_SCALE;
        
        // Update dashboard status for the 180-degree macro
        turningStatusEntry.setBoolean(turning180);

        // --- 1. D-Pad Nudge/Bump Control Logic (Highest Priority) ---
        // Checks the Driver's D-Pad (POV) for precise, slow-speed movement.
        int povAngle = driver.getPOV();

        if (povAngle != -1) {
            // If the D-Pad is pressed, use the fixed NUDGE speed and bypass analog stick control
            switch (povAngle) {
                case 0: // Up: Forward Nudge
                    drive.arcadeDrive(NUDGE_SPEED, 0.0);
                    break;
                case 180: // Down: Reverse Nudge
                    drive.arcadeDrive(-NUDGE_SPEED, 0.0);
                    break;
                case 270: // Left: Turn Left Nudge
                    drive.arcadeDrive(0.0, NUDGE_SPEED); 
                    break;
                case 90: // Right: Turn Right Nudge
                    drive.arcadeDrive(0.0, -NUDGE_SPEED); 
                    break;
                case 45: // Up-Right: Forward + Turn Right Nudge
                    drive.arcadeDrive(NUDGE_SPEED, -NUDGE_SPEED);
                    break;
                case 135: // Down-Right: Reverse + Turn Right Nudge
                    drive.arcadeDrive(-NUDGE_SPEED, -NUDGE_SPEED);
                    break;
                case 225: // Down-Left: Reverse + Turn Left Nudge
                    drive.arcadeDrive(-NUDGE_SPEED, NUDGE_SPEED);
                    break;
                case 315: // Up-Left: Forward + Turn Left Nudge
                    drive.arcadeDrive(NUDGE_SPEED, NUDGE_SPEED);
                    break;
                default:
                    drive.stopMotor();
                    break;
            }
            // Crucial: Stop processing drive inputs from analog sticks if D-Pad is active
            return; 
        }

        // --- 2. 180 Turn Macro Logic --- (Driver only)
        // Check if the macro is triggered and not already running
        if (driver.getAButtonPressed() && !turning180) {
            turning180 = true;
            turnTimer.reset();
            turnTimer.start();
        }

        if (turning180) {
            if (turnTimer.get() < TURN_TIME) {
                // Execute a tank drive turn-in-place at TURN_SPEED
                drive.tankDrive(TURN_SPEED, -TURN_SPEED);
                return; // Prevents subsequent drive code from overriding the turn
            } else {
                // Macro complete
                drive.stopMotor();
                turning180 = false;
                turnTimer.stop();
            }
        }

        // --- 3. Control Mode Selection and Input Reading ---
        String controlMode = controlModeChooser.getSelected();
        if (controlMode == null) controlMode = SINGLE_OPERATOR; 

        // Determine which controller handles the mechanism inputs
        XboxController mechController = (DUAL_OPERATOR.equals(controlMode)) ? operator : driver;

        // Initialize all movement variables
        double forward = 0;
        double turn = 0;
        double left = 0;
        double right = 0;
        double elevatorSpeed = 0;

        // --- Drive and Mechanism Input Mapping ---
        if (SINGLE_OPERATOR.equals(controlMode)) {
            // Drive inputs (Driver's Left Y/X for Arcade, Left/Right Y for Tank)
            forward = -driver.getLeftY(); 
            turn = -driver.getLeftX(); 
            left = -driver.getLeftY(); 
            right = -driver.getRightY();
            
            // Mechanism inputs (from Driver's Triggers)
            double rightTrigger = driver.getRightTriggerAxis();
            double leftTrigger = driver.getLeftTriggerAxis();
            // Up is Right Trigger (positive command), Down is Left Trigger (negative command)
            elevatorSpeed = (rightTrigger - leftTrigger) * ELEVATOR_MAX_SPEED;

            // Update Trigger values on Shuffleboard for debugging
            leftTriggerEntry.setDouble(leftTrigger);
            rightTriggerEntry.setDouble(rightTrigger);

        } else if (DUAL_OPERATOR.equals(controlMode)) {
            // Drive inputs (from Driver only)
            forward = -driver.getLeftY(); 
            turn = -driver.getLeftX(); 
            left = -driver.getLeftY(); 
            right = -driver.getRightY(); 
            
            // Mechanism inputs (from Operator's Triggers)
            double rightTrigger = operator.getRightTriggerAxis();
            double leftTrigger = operator.getLeftTriggerAxis();
            elevatorSpeed = (rightTrigger - leftTrigger) * ELEVATOR_MAX_SPEED;

            leftTriggerEntry.setDouble(leftTrigger);
            rightTriggerEntry.setDouble(rightTrigger);
        }

        // ---------------------------------------------------------------------
        // --- 4. Variable Speed Manipulator Control Logic ---
        // ---------------------------------------------------------------------
        double manipulatorInput = mechController.getRightY(); // Right Y-Axis controls Intake/Output
        double manipulatorOutput = 0.0;
        
        if (Math.abs(manipulatorInput) > DEADBAND) {
            if (manipulatorInput < 0) {
                // Stick Forward (Negative Y): Intake
                // Output is positive, scaled by INTAKE_SPEED
                manipulatorOutput = -manipulatorInput * INTAKE_SPEED; 
                manipulatorStatusEntry.setString("INTAKE (Prop: " + String.format("%.2f", manipulatorOutput) + ")");
            } else {
                // Stick Backward (Positive Y): Output/Eject
                // Output is negative, scaled by OUTPUT_SPEED (which is already negative)
                manipulatorOutput = manipulatorInput * OUTPUT_SPEED;
                manipulatorStatusEntry.setString("OUTPUT (Prop: " + String.format("%.2f", manipulatorOutput) + ")");
            }
        } else {
            // Stick within deadband: Motor off
            manipulatorOutput = 0.0;
            manipulatorStatusEntry.setString("OFF");
        }

        manipulatorMotor.set(manipulatorOutput);


        // --- Drive Input Post-Processing (Executed BEFORE sending to DifferentialDrive) ---

        // 1. Apply Deadband to eliminate stick drift
        forward = applyDeadband(forward, DEADBAND);
        turn = applyDeadband(turn, DEADBAND);
        left = applyDeadband(left, DEADBAND);
        right = applyDeadband(right, DEADBAND);
        
        // 2. Apply Input Shaping (Cubing) for finer control near the center
        // Output = Input * Input * Input. This slows down low stick deflections.
        forward = forward * forward * forward;
        turn = turn * turn * turn; 
        left = left * left * left;
        right = right * right * right;
        
        // --- 5. Drive Mode Execution ---
        String mode = driveModeChooser.getSelected();
        if (mode == null) mode = DRIVE_ARCADE;
        currentDriveModeEntry.setString(mode); // Update dashboard

        switch (mode) {
            case DRIVE_TANK:
                // TANK drive uses the Left and Right Y axes (separate sticks)
                // Apply the overall speed scale to the output
                drive.tankDrive(left * speedScale, right * speedScale);
                break;
            case DRIVE_CURVATURE:
                // CURVATURE drive uses Left Stick (Y for speed, X for turn). Boolean true enables quick turn.
                drive.curvatureDrive(forward * speedScale, turn * turnScale, true);
                break;
            case DRIVE_ARCADE:
            default:
                // ARCADE drive uses Left Stick (Y for speed, X for turn).
                drive.arcadeDrive(forward * speedScale, turn * turnScale);
                break;
        }

        // --- 6. Elevator Control Execution ---
        
        // Apply deadband to the mechanism input as well
        if (Math.abs(elevatorSpeed) < 0.05) elevatorSpeed = 0;
        
        // Set motor speed (Note: elevatorSpeed already includes ELEVATOR_MAX_SPEED scaling)
        elevatorMotor.set(elevatorSpeed);
        
        // Update dashboard with the scaled output percentage (0 to 100)
        elevatorOutputEntry.setDouble(elevatorSpeed * 100.0 / ELEVATOR_MAX_SPEED); 
    }
    
    /**
     * Helper method to apply a deadband to a joystick input value.
     * Returns 0.0 if the absolute value is less than the deadband threshold.
     */
    private double applyDeadband(double value, double deadband) {
        return Math.abs(value) < deadband ? 0.0 : value;
    }

    /**
     * Utility function to set the neutral mode (Brake or Coast) for all four drive motors.
     */
    private void setNeutralMode(NeutralMode mode) {
        leftFront.setNeutralMode(mode);
        leftRear.setNeutralMode(mode);
        rightFront.setNeutralMode(mode);
        rightRear.setNeutralMode(mode);
    }

    /**
     * Runs once when the robot enters disabled mode.
     * Used for final diagnostics and setting a safe state (Coast mode).
     */
    @Override
    public void disabledInit() {
        // Set motors to Coast mode so they can be pushed easily
        setNeutralMode(NeutralMode.Coast);
        drive.stopMotor();
        
        // Disabled Current Diagnostic Check
        double totalCurrent = pdDevice.getTotalCurrent();
        if (totalCurrent > 5.0) { 
            // Warning if total current draw is high (indicates a stuck motor, short, etc.)
            motorCurrentsDisabledEntry.setString("WARNING: High Disabled Current (" + String.format("%.1f", totalCurrent) + " A)! Check wiring.");
        } else {
            // Status OK
            motorCurrentsDisabledEntry.setString("Disabled Current Check: OK (Total: " + String.format("%.1f", totalCurrent) + " A)");
        }
    }
    
    /**
     * Runs once when the robot enters autonomous mode.
     * Used for reading configurations and resetting timers.
     */
    @Override
    public void autonomousInit() {
        selectedAuto = autoChooser.getSelected(); // Get routine selection from Shuffleboard
        selectedAutoEntry.setString(selectedAuto);
        autoDriveTime = autoDriveTimeEntry.getDouble(2.0); // Get drive duration from Shuffleboard (default 2.0s)
        autoTimer.reset();
        autoTimer.start();
        autoStatusEntry.setString("Starting Autonomous: " + selectedAuto);
        setNeutralMode(NeutralMode.Brake); // Set to Brake mode for precise movement
        drive.stopMotor();
    }
    
    /**
     * Runs continuously during autonomous mode (Time-based state machine).
     */
    @Override
    public void autonomousPeriodic() {
        double remainingTime = autoDriveTime - autoTimer.get();
        String status = "Time Remaining: " + String.format("%.2f", remainingTime) + "s";
        
        // Autonomous Execution Logic (Simple Switch/Case for Time-Based Routines)
        switch (selectedAuto) {
            case AUTO_TURN:
                if (autoTimer.get() < TURN_TIME) {
                    // Turn in place at 50% speed for the calibrated duration
                    drive.tankDrive(TURN_SPEED, -TURN_SPEED);
                    autoStatusEntry.setString("Executing Turn 180°. " + status);
                } else {
                    // Turn finished
                    drive.stopMotor();
                    autoStatusEntry.setString("Turn Complete. Motor Stopped.");
                }
                break;

            case AUTO_DEFAULT:
            default:
                if (autoTimer.get() < autoDriveTime) {
                    // Drive forward at 50% speed for the configurable duration
                    drive.arcadeDrive(0.5, 0.0);
                    autoStatusEntry.setString("Driving Forward. " + status);
                } else {
                    // Drive finished
                    drive.stopMotor();
                    autoStatusEntry.setString("Drive Complete. Motor Stopped.");
                }
                break;
            }
    }
}