package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private final SparkMax motor = new SparkMax(Constants.CAN_ID_ELEVATOR, MotorType.kBrushless);
    private final DigitalInput bottomLimitSwitch = new DigitalInput(Constants.DIO_ELEVATOR_BOTTOM_LIMIT);
    private final DigitalInput topLimitSwitch = new DigitalInput(Constants.DIO_ELEVATOR_TOP_LIMIT);
    
    // State variables
    private double currentSpeed = 0.0;
    private String status = "OFF";
    
    public Elevator() {
        // Any elevator-specific initialization can go here
    }
    
    public void setSpeed(double speed) {
        // Apply deadband
        if (Math.abs(speed) < 0.05) {
            speed = 0;
        }
        
        // Apply limit switch safety
        boolean isAtBottom = bottomLimitSwitch.get();
        boolean isAtTop = topLimitSwitch.get();
        
        if (isAtBottom && speed < 0) {
            speed = 0; // Stop downward motion at bottom limit
        }
        
        if (isAtTop && speed > 0) {
            speed = 0; // Stop upward motion at top limit
        }
        
        // Scale by max speed and set motor
        currentSpeed = speed * Constants.ELEVATOR_MAX_SPEED;
        motor.set(currentSpeed);
        
        // Update status
        if (currentSpeed > 0) {
            status = "MOVING UP";
        } else if (currentSpeed < 0) {
            status = "MOVING DOWN";
        } else {
            status = "STOPPED";
        }
    }
    
    public void stop() {
        setSpeed(0);
    }
    
    // Getters for dashboard and logic
    public double getCurrent() {
        return motor.getOutputCurrent();
    }
    
    public double getOutput() {
        return currentSpeed * 100.0; // Convert to percentage for display
    }
    
    public boolean isAtBottom() {
        return bottomLimitSwitch.get();
    }
    
    public boolean isAtTop() {
        return topLimitSwitch.get();
    }
    
    public String getStatus() {
        return status;
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // You can add any periodic logic here
    }
}