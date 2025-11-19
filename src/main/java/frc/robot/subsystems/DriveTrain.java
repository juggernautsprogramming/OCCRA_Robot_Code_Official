package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
// DriveTrain.java
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// *** FINAL CORRECT IMPORT FOR VELOCITY FIX ***
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod; 
// **********************************
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
    private final WPI_TalonSRX leftFront = new WPI_TalonSRX(Constants.CAN_ID_LEFT_FRONT);
    private final WPI_TalonSRX leftRear = new WPI_TalonSRX(Constants.CAN_ID_LEFT_REAR);
    private final WPI_TalonSRX rightFront = new WPI_TalonSRX(Constants.CAN_ID_RIGHT_FRONT);
    private final WPI_TalonSRX rightRear = new WPI_TalonSRX(Constants.CAN_ID_RIGHT_REAR);
    
    private final DifferentialDrive drive = new DifferentialDrive(leftFront, rightFront);
    private boolean isTurning180 = false;
    
    public void initialize() {
        configureMotors();
        resetEncoders(); 
    }
    
    private void configureMotors() {
        leftFront.configFactoryDefault();
        leftRear.configFactoryDefault();
        rightFront.configFactoryDefault();
        rightRear.configFactoryDefault();
        
        leftFront.configOpenloopRamp(0.25);
        leftRear.configOpenloopRamp(0.25);
        rightFront.configOpenloopRamp(0.25);
        rightRear.configOpenloopRamp(0.25);
        
        // **ENCODER ADDITION:** Configure the encoders on the leader motors
        leftFront.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        rightFront.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        
        // ******************************************************************************
        // *** FINAL FIX: USE THE CORRECT, NON-DEPRECATED ENUM ***
        // This resolves the compilation error: configVelocityMeasurementPeriod(SensorVelocityMeasPeriod, int)
        
        leftFront.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_100Ms, 10); 
        leftFront.configVelocityMeasurementWindow(10, 10); 

        rightFront.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_100Ms, 10); 
        rightFront.configVelocityMeasurementWindow(10, 10); 
        // ******************************************************************************
        
        leftFront.setSensorPhase(true); 
        rightFront.setSensorPhase(true); 

        setNeutralMode(NeutralMode.Brake);
        
        leftRear.follow(leftFront);
        rightRear.follow(rightFront);
        
        rightFront.setInverted(true);
        rightRear.setInverted(true);
        
        rightFront.setSensorPhase(true); 
        
        SupplyCurrentLimitConfiguration limit = new SupplyCurrentLimitConfiguration(true, 40, 60, 0.1);
        leftFront.configSupplyCurrentLimit(limit);
        leftRear.configSupplyCurrentLimit(limit);
        rightFront.configSupplyCurrentLimit(limit);
        rightRear.configSupplyCurrentLimit(limit);
    }
    
    public void arcadeDrive(double forward, double turn) {
        drive.arcadeDrive(forward * Constants.SPEED_SCALE, turn * Constants.TURN_SCALE);
    }
    
    public void tankDrive(double left, double right) {
        drive.tankDrive(left * Constants.SPEED_SCALE, right * Constants.SPEED_SCALE);
    }
    
    public void curvatureDrive(double forward, double turn, boolean quickTurn) {
        drive.curvatureDrive(forward * Constants.SPEED_SCALE, turn * Constants.TURN_SCALE, quickTurn);
    }
    
    public void nudgeDrive(int povAngle) {
        switch (povAngle) {
            case 0: drive.arcadeDrive(Constants.NUDGE_SPEED, 0.0); break;
            case 180: drive.arcadeDrive(-Constants.NUDGE_SPEED, 0.0); break;
            case 270: drive.arcadeDrive(0.0, Constants.NUDGE_SPEED); break;
            case 90: drive.arcadeDrive(0.0, -Constants.NUDGE_SPEED); break;
            case 45: drive.arcadeDrive(Constants.NUDGE_SPEED, -Constants.NUDGE_SPEED); break;
            case 135: drive.arcadeDrive(-Constants.NUDGE_SPEED, -Constants.NUDGE_SPEED); break;
            case 225: drive.arcadeDrive(-Constants.NUDGE_SPEED, Constants.NUDGE_SPEED); break;
            case 315: drive.arcadeDrive(Constants.NUDGE_SPEED, Constants.NUDGE_SPEED); break;
            default: stop(); break;
        }
    }
    
    public void turn180() {
        drive.tankDrive(Constants.TURN_SPEED, -Constants.TURN_SPEED);
    }
    
    public void stop() {
        drive.stopMotor();
    }

    public double getLeftEncoderPosition() {
        return leftFront.getSelectedSensorPosition(0);
    }

    public double getRightEncoderPosition() {
        return rightFront.getSelectedSensorPosition(0);
    }

    public double getLeftEncoderVelocity() {
        return leftFront.getSelectedSensorVelocity(0);
    }

    public double getRightEncoderVelocity() {
        return rightFront.getSelectedSensorVelocity(0);
    }

    public void resetEncoders() {
        leftFront.setSelectedSensorPosition(0, 0, 10);
        rightFront.setSelectedSensorPosition(0, 0, 10);
    }

    public double getLeftOutput() { return leftFront.getMotorOutputPercent() * 100.0; }
    public double getRightOutput() { return rightFront.getMotorOutputPercent() * 100.0; }
    public double getLeftCurrent() { return leftFront.getSupplyCurrent() + leftRear.getSupplyCurrent(); }
    public double getRightCurrent() { return rightFront.getSupplyCurrent() + rightRear.getSupplyCurrent(); }
    
    public void setNeutralMode(NeutralMode mode) {
        leftFront.setNeutralMode(mode);
        leftRear.setNeutralMode(mode);
        rightFront.setNeutralMode(mode);
        rightRear.setNeutralMode(mode);
    }
    
    public void setTurning180(boolean turning) { isTurning180 = turning; }
    public boolean isTurning180() { return isTurning180; }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber(
            "Drive/Left Encoder Position (Ticks)", 
            getLeftEncoderPosition()
        );
        SmartDashboard.putNumber(
            "Drive/Right Encoder Position (Ticks)", 
            getRightEncoderPosition()
        );

        SmartDashboard.putNumber(
            "Drive/Left Motor Current", 
            getLeftCurrent()
        );
    }
}