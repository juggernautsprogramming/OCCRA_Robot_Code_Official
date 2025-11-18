package frc.robot;

// --- CTRE Imports ---
import com.ctre.phoenix.motorcontrol.NeutralMode;

// --- REV Robotics Imports ---
// (Now handled in subsystems)

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.PowerDistribution;
// --- WPILib Core Imports ---
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// --- Dashboard/GUI Imports (Shuffleboard/SmartDashboard) ---
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

// --- Subsystem Imports ---
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;

/**
 * This class is the primary interface for operating the FRC robot.
 * It extends TimedRobot, providing the main control loop structure.
 * It uses subsystem classes to organize hardware and logic.
 */
public class Robot extends TimedRobot {

    // --- Subsystem Definitions ---
    private final DriveTrain driveTrain = new DriveTrain();
    private final Elevator elevator = new Elevator();
    private final Manipulator manipulator = new Manipulator();

    // --- Hardware Definitions ---
    private final PowerDistribution pdDevice = new PowerDistribution();

    // --- Timers ---
    private final Timer autoTimer = new Timer();
    private final Timer turnTimer = new Timer();

    // --- Controllers ---
    private final XboxController driver = new XboxController(0);
    private final XboxController operator = new XboxController(1);

    // --- Autonomous Setup ---
    private SendableChooser<String> autoChooser = new SendableChooser<>();
    private static final String AUTO_DEFAULT = "Drive Forward";
    private static final String AUTO_TURN = "Turn 180°";
    private String selectedAuto;
    private double autoDriveTime;

    // --- Control Mode Constants ---
    private static final String SINGLE_OPERATOR = "Solo";
    private static final String DUAL_OPERATOR = "Co-Op";
    
    // --- Drive Mode Constants ---
    private static final String DRIVE_ARCADE = "Arcade";
    private static final String DRIVE_TANK = "Tank";
    private static final String DRIVE_CURVATURE = "Curvature";

    // --- Shuffleboard Entries ---
    // General/Auto Tab
    private GenericEntry autoDriveTimeEntry;
    private GenericEntry batteryVoltageEntry;
    private GenericEntry selectedAutoEntry;
    private GenericEntry autoStatusEntry;

    // Drive Tab
    private GenericEntry leftDriveCurrentEntry;
    private GenericEntry rightDriveCurrentEntry;
    private GenericEntry leftDriveOutputEntry;
    private GenericEntry rightDriveOutputEntry;
    private GenericEntry turningStatusEntry;
    private GenericEntry bottomLimitEntry;
    private GenericEntry topLimitEntry;

    // Mechanism Tab
    private GenericEntry elevatorCurrentEntry;
    private GenericEntry manipulatorCurrentEntry;
    private GenericEntry manipulatorStatusEntry;
    private GenericEntry elevatorOutputEntry;
    private GenericEntry leftTriggerEntry;
    private GenericEntry rightTriggerEntry;

    // Warning and Mode Display
    private GenericEntry batteryWarningEntryDrive;
    private GenericEntry batteryWarningEntryDisabled;
    private GenericEntry currentDriveModeEntry;
    private GenericEntry motorCurrentsDisabledEntry;

    // --- Choosers ---
    private final SendableChooser<String> controlModeChooser = new SendableChooser<>();
    private final SendableChooser<String> driveModeChooser = new SendableChooser<>();

    // --- Shuffleboard Tabs ---
    private ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");
    private ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");
    private ShuffleboardTab disabledTab = Shuffleboard.getTab("Disabled");

    /**
     * This function is run when the robot is first started (once only).
     */
    @Override
    public void robotInit() {
        // Initialize subsystems
        driveTrain.initialize();
        
        setupShuffleboard();
    }

    /**
     * Sets up all Shuffleboard widgets and tabs
     */
    private void setupShuffleboard() {
        // Setup Choosers
        driveModeChooser.setDefaultOption(DRIVE_ARCADE, DRIVE_ARCADE);
        driveModeChooser.addOption(DRIVE_TANK, DRIVE_TANK);
        driveModeChooser.addOption(DRIVE_CURVATURE, DRIVE_CURVATURE);

        controlModeChooser.setDefaultOption(SINGLE_OPERATOR, SINGLE_OPERATOR);
        controlModeChooser.addOption(DUAL_OPERATOR, DUAL_OPERATOR);

        // =======================================================================
        // Drive Tab Setup
        // =======================================================================

        // Row 0: Modes and Status
        driveTab.add("Control Mode", controlModeChooser)
                .withWidget(BuiltInWidgets.kComboBoxChooser)
                .withPosition(0, 0).withSize(3, 1);
                
        driveTab.add("Drive Mode", driveModeChooser)
                .withWidget(BuiltInWidgets.kComboBoxChooser)
                .withPosition(3, 0).withSize(3, 1);

        currentDriveModeEntry = driveTab.add("Current Drive Mode", DRIVE_ARCADE)
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(6, 0).withSize(3, 1)
                .getEntry();

        turningStatusEntry = driveTab.add("180 Turn Active", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(9, 0).withSize(1, 1)
                .getEntry();
        
        // Row 1: Drive Train Live Stats
        leftDriveCurrentEntry = driveTab.add("Left Current (A)", 0.0)
                .withWidget(BuiltInWidgets.kVoltageView)
                .withPosition(0, 1).withSize(3, 1)
                .getEntry();

        rightDriveCurrentEntry = driveTab.add("Right Current (A)", 0.0)
                .withWidget(BuiltInWidgets.kVoltageView)
                .withPosition(3, 1).withSize(3, 1)
                .getEntry();

        leftDriveOutputEntry = driveTab.add("Left Output (%)", 0.0)
                .withWidget(BuiltInWidgets.kDial)
                .withPosition(6, 1).withSize(2, 1)
                .getEntry();

        rightDriveOutputEntry = driveTab.add("Right Output (%)", 0.0)
                .withWidget(BuiltInWidgets.kDial)
                .withPosition(8, 1).withSize(2, 1)
                .getEntry();

        // Row 2: Mechanism Stats
        elevatorOutputEntry = driveTab.add("Elevator Output (%)", 0)
                .withWidget(BuiltInWidgets.kDial)
                .withPosition(0, 2).withSize(2, 1)
                .getEntry();

        elevatorCurrentEntry = driveTab.add("Elevator Current (A)", 0.0)
                .withWidget(BuiltInWidgets.kNumberBar)
                .withPosition(2, 2).withSize(2, 1)
                .getEntry();

        manipulatorCurrentEntry = driveTab.add("Manipulator Current (A)", 0.0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withPosition(4, 2).withSize(3, 1)
            .getEntry();
        
        manipulatorStatusEntry = driveTab.add("Manipulator Status", "OFF")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(7, 2).withSize(3, 1)
            .getEntry();

        // Row 3: Trigger Inputs and Battery Warning
        leftTriggerEntry = driveTab.add("Left Trigger (Elevator Down)", 0)
                .withWidget(BuiltInWidgets.kNumberBar)
                .withPosition(0, 3).withSize(3, 1)
                .getEntry();

        rightTriggerEntry = driveTab.add("Right Trigger (Elevator Up)", 0)
                .withWidget(BuiltInWidgets.kNumberBar)
                .withPosition(3, 3).withSize(3, 1)
                .getEntry();
                
        batteryWarningEntryDrive = driveTab.add("Battery Status", "OK")
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(6, 3).withSize(4, 1)
                .getEntry();
                
        // Row 4: Limit Switches
        bottomLimitEntry = driveTab.add("Bottom Limit Hit", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(0, 4).withSize(2, 1)
                .getEntry();
                
        topLimitEntry = driveTab.add("Top Limit Hit", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(2, 4).withSize(2, 1)
                .getEntry();

        // =======================================================================
        // Autonomous Tab Setup
        // =======================================================================

        autoChooser.setDefaultOption("Drive Forward", AUTO_DEFAULT);
        autoChooser.addOption("Turn 180°", AUTO_TURN);

        autoTab.add("1. Select Autonomous Mode", autoChooser)
           .withPosition(0, 0).withSize(3, 1);

        autoDriveTimeEntry = autoTab.add("2. Auto Drive Time (s)", 2.0)
            .withPosition(3, 0).withSize(2, 1)
            .getEntry();

        batteryVoltageEntry = autoTab.add("Battery Voltage (V)", 12.5)
            .withWidget(BuiltInWidgets.kVoltageView)
            .withPosition(5, 0).withSize(3, 1)
            .getEntry();

        selectedAutoEntry = autoTab.add("Selected Auto", AUTO_DEFAULT)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(8, 0).withSize(2, 1)
            .getEntry();
        
        autoStatusEntry = autoTab.add("Auto Status", "Ready for Init")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 1).withSize(10, 1)
            .getEntry();

        // =======================================================================
        // Disabled Tab Setup
        // =======================================================================

        batteryWarningEntryDisabled = disabledTab.add("Battery Status", "OK")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 0).withSize(10, 1)
            .getEntry();
            
        motorCurrentsDisabledEntry = disabledTab.add("Disabled Current Check", "Checking...")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 1).withSize(10, 1)
            .getEntry();
    }

    /**
     * This function runs at a constant rate regardless of the robot's mode.
     */
    @Override
    public void robotPeriodic() {
        // Update battery voltage and warnings
        updateBatteryStatus();
        
        // Update dashboard with subsystem data
        updateDashboard();
        
        // Run periodic methods for all subsystems
        driveTrain.periodic();
        elevator.periodic();
        manipulator.periodic();
    }

    /**
     * Updates battery status and warnings
     */
    private void updateBatteryStatus() {
        double voltage = pdDevice.getVoltage();
        batteryVoltageEntry.setDouble(voltage);

        if (voltage < 10.5) { 
            String warning = "!!! LOW BATTERY: " + String.format("%.2f", voltage) + " V !!!";
            batteryWarningEntryDrive.setString(warning);
            batteryWarningEntryDisabled.setString(warning);
        } else {
            batteryWarningEntryDrive.setString("Battery OK: " + String.format("%.2f", voltage) + " V");
            batteryWarningEntryDisabled.setString("Battery OK: " + String.format("%.2f", voltage) + " V");
        }
    }

    /**
     * Updates all dashboard entries with current subsystem states
     */
    private void updateDashboard() {
        // Drive train stats
        leftDriveOutputEntry.setDouble(driveTrain.getLeftOutput());
        rightDriveOutputEntry.setDouble(driveTrain.getRightOutput());
        leftDriveCurrentEntry.setDouble(driveTrain.getLeftCurrent());
        rightDriveCurrentEntry.setDouble(driveTrain.getRightCurrent());
        turningStatusEntry.setBoolean(driveTrain.isTurning180());

        // Elevator stats
        elevatorOutputEntry.setDouble(elevator.getOutput());
        elevatorCurrentEntry.setDouble(elevator.getCurrent());
        bottomLimitEntry.setBoolean(elevator.isAtBottom());
        topLimitEntry.setBoolean(elevator.isAtTop());

        // Manipulator stats
        manipulatorCurrentEntry.setDouble(manipulator.getCurrent());
        manipulatorStatusEntry.setString(manipulator.getStatus());
    }

    /**
     * This function is called continuously during teleoperated mode.
     */
    @Override
    public void teleopPeriodic() {
        // 1. D-Pad Nudge Control (Highest Priority)
        if (handleDPadControl()) {
            return;
        }

        // 2. 180 Turn Macro
        if (handle180TurnMacro()) {
            return;
        }

        // 3. Normal Drive Control
        handleDriveControl();

        // 4. Mechanism Control
        handleMechanismControl();
    }

    /**
     * Handles D-Pad nudge control
     * @return true if D-Pad is active, false otherwise
     */
    private boolean handleDPadControl() {
        int povAngle = driver.getPOV();
        if (povAngle != -1) {
            driveTrain.nudgeDrive(povAngle);
            return true;
        }
        return false;
    }

    /**
     * Handles 180-degree turn macro
     * @return true if turn macro is active, false otherwise
     */
    private boolean handle180TurnMacro() {
        // Start turn macro
        if (driver.getAButtonPressed() && !driveTrain.isTurning180()) {
            driveTrain.setTurning180(true);
            turnTimer.reset();
            turnTimer.start();
        }

        // Execute turn macro
        if (driveTrain.isTurning180()) {
            if (turnTimer.get() < Constants.TURN_TIME) {
                driveTrain.turn180();
                return true; // Skip other drive controls
            } else {
                // Macro complete
                driveTrain.stop();
                driveTrain.setTurning180(false);
                turnTimer.stop();
            }
        }
        return false;
    }

    /**
     * Handles normal drive control based on selected drive mode
     */
    private void handleDriveControl() {
        String controlMode = controlModeChooser.getSelected();
        if (controlMode == null) controlMode = SINGLE_OPERATOR;

        // Get raw inputs
        double forward = -driver.getLeftY();
        double turn = -driver.getLeftX();
        double left = -driver.getLeftY();
        double right = -driver.getRightY();

        // Apply processing
        forward = applyDeadband(forward, Constants.DEADBAND);
        turn = applyDeadband(turn, Constants.DEADBAND);
        left = applyDeadband(left, Constants.DEADBAND);
        right = applyDeadband(right, Constants.DEADBAND);
        
        forward = forward * forward * forward;
        turn = turn * turn * turn;
        left = left * left * left;
        right = right * right * right;

        // Execute drive mode
        String driveMode = driveModeChooser.getSelected();
        if (driveMode == null) driveMode = DRIVE_ARCADE;
        currentDriveModeEntry.setString(driveMode);

        switch (driveMode) {
            case DRIVE_TANK:
                driveTrain.tankDrive(left, right);
                break;
            case DRIVE_CURVATURE:
                driveTrain.curvatureDrive(forward, turn, true);
                break;
            case DRIVE_ARCADE:
            default:
                driveTrain.arcadeDrive(forward, turn);
                break;
        }
    }

    /**
     * Handles elevator and manipulator control
     */
    private void handleMechanismControl() {
        String controlMode = controlModeChooser.getSelected();
        if (controlMode == null) controlMode = SINGLE_OPERATOR;

        XboxController mechController = (DUAL_OPERATOR.equals(controlMode)) ? operator : driver;

        // Elevator control
        double rightTrigger = mechController.getRightTriggerAxis();
        double leftTrigger = mechController.getLeftTriggerAxis();
        double elevatorSpeed = rightTrigger - leftTrigger;
        
        elevator.setSpeed(elevatorSpeed);

        // Update trigger displays
        leftTriggerEntry.setDouble(leftTrigger);
        rightTriggerEntry.setDouble(rightTrigger);

        // Manipulator control
        double manipulatorInput = mechController.getRightY();
        manipulator.setSpeed(manipulatorInput);
    }

    /**
     * Helper method to apply a deadband to a joystick input value.
     */
    private double applyDeadband(double value, double deadband) {
        return Math.abs(value) < deadband ? 0.0 : value;
    }

    /**
     * Runs once when the robot enters disabled mode.
     */
    @Override
    public void disabledInit() {
        // Set motors to Coast mode so they can be pushed easily
        driveTrain.setNeutralMode(NeutralMode.Coast);
        driveTrain.stop();
        
        // Stop all mechanisms
        elevator.stop();
        manipulator.stop();
        
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
        
        driveTrain.setNeutralMode(NeutralMode.Brake);
        driveTrain.stop();
    }

    /**
     * Runs continuously during autonomous mode.
     */
    @Override
    public void autonomousPeriodic() {
        double remainingTime = autoDriveTime - autoTimer.get();
        String status = "Time Remaining: " + String.format("%.2f", remainingTime) + "s";
        
        switch (selectedAuto) {
            case AUTO_TURN:
                if (autoTimer.get() < Constants.TURN_TIME) {
                    driveTrain.turn180();
                    autoStatusEntry.setString("Executing Turn 180°. " + status);
                } else {
                    driveTrain.stop();
                    autoStatusEntry.setString("Turn Complete. Motor Stopped.");
                }
                break;

            case AUTO_DEFAULT:
            default:
                if (autoTimer.get() < autoDriveTime) {
                    driveTrain.arcadeDrive(0.5, 0.0);
                    autoStatusEntry.setString("Driving Forward. " + status);
                } else {
                    driveTrain.stop();
                    autoStatusEntry.setString("Drive Complete. Motor Stopped.");
                }
                break;
        }
    }

    /**
     * Runs once when teleop starts
     */
    @Override
    public void teleopInit() {
        driveTrain.setNeutralMode(NeutralMode.Brake);
        driveTrain.stop();
        elevator.stop();
        manipulator.stop();
    }

    /**
     * Runs once when autonomous ends
     */
    @Override
    public void autonomousExit() {
        driveTrain.stop();
        elevator.stop();
        manipulator.stop();
    }

    /**
     * Runs once when teleop ends
     */
    @Override
    public void teleopExit() {
        driveTrain.stop();
        elevator.stop();
        manipulator.stop();
    }
}