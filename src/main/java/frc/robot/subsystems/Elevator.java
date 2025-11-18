package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private SparkMax motor;
    private final DigitalInput bottomLimitSwitch;
    private final DigitalInput topLimitSwitch;
    
    private double currentSpeed = 0.0;
    private String status = "OFF";
    private boolean motorInitialized = false;
    private double maxCurrentSeen = 0.0;
    
    public Elevator() {
        try {
            System.out.println("Initializing elevator motor on CAN ID: " + Constants.CAN_ID_ELEVATOR);
            motor = new SparkMax(Constants.CAN_ID_ELEVATOR, MotorType.kBrushless);
            motorInitialized = true;
            System.out.println("Elevator motor initialized successfully");
        } catch (Exception e) {
            System.err.println("FAILED to initialize elevator motor: " + e.getMessage());
            motorInitialized = false;
            motor = null;
        }
        
        try {
            bottomLimitSwitch = new DigitalInput(Constants.DIO_ELEVATOR_BOTTOM_LIMIT);
            topLimitSwitch = new DigitalInput(Constants.DIO_ELEVATOR_TOP_LIMIT);
            System.out.println("Limit switches initialized on DIO: " + 
                Constants.DIO_ELEVATOR_BOTTOM_LIMIT + ", " + Constants.DIO_ELEVATOR_TOP_LIMIT);
            System.out.println("INITIAL LIMIT STATES - Bottom: " + bottomLimitSwitch.get() + " Top: " + topLimitSwitch.get());
            System.out.println("NOTE: Limit switches will be INVERTED in logic (!get())");
        } catch (Exception e) {
            System.err.println("FAILED to initialize limit switches: " + e.getMessage());
            throw e;
        }
    }
    
    public void setSpeed(double speed) {
        if (!motorInitialized) {
            System.err.println("Elevator motor not initialized - cannot set speed");
            status = "MOTOR NOT INITIALIZED";
            return;
        }
        
        // Apply deadband
        if (Math.abs(speed) < 0.05) {
            speed = 0;
        }
        
        // Apply limit switch safety WITH INVERTED LOGIC
        boolean isAtBottom = !bottomLimitSwitch.get(); // INVERTED: true when switch is pressed
        boolean isAtTop = !topLimitSwitch.get();       // INVERTED: true when switch is pressed
        
        System.out.println("Elevator Command - Speed: " + speed + 
                          ", Bottom Limit: " + isAtBottom + " (raw: " + bottomLimitSwitch.get() + ")" +
                          ", Top Limit: " + isAtTop + " (raw: " + topLimitSwitch.get() + ")");
        
        // Safety limits with inverted logic
        if (isAtBottom && speed < 0) {
            speed = 0;
            System.out.println("SAFETY: At bottom limit, cannot move down");
        }
        
        if (isAtTop && speed > 0) {
            speed = 0;
            System.out.println("SAFETY: At top limit, cannot move up");
        }
        
        // Scale by max speed and set motor
        currentSpeed = speed * Constants.ELEVATOR_MAX_SPEED;
        
        try {
            motor.set(currentSpeed);
            
            // Update status
            if (currentSpeed > 0) {
                status = "MOVING UP: " + String.format("%.1f", currentSpeed * 100) + "%";
            } else if (currentSpeed < 0) {
                status = "MOVING DOWN: " + String.format("%.1f", currentSpeed * 100) + "%";
            } else {
                status = "STOPPED";
            }
            
        } catch (Exception e) {
            System.err.println("Error setting elevator motor speed: " + e.getMessage());
            status = "ERROR: " + e.getMessage();
        }
    }
    
    public void stop() { 
        setSpeed(0); 
    }
    
    public double getCurrent() { 
        if (!motorInitialized) return 0;
        try {
            double current = motor.getOutputCurrent();
            if (current > maxCurrentSeen) {
                maxCurrentSeen = current;
            }
            return current;
        } catch (Exception e) {
            System.err.println("Error reading elevator current: " + e.getMessage());
            return 0;
        }
    }
    
    public double getMaxCurrentSeen() {
        return maxCurrentSeen;
    }
    
    public void resetMaxCurrent() {
        maxCurrentSeen = 0;
    }
    
    public double getOutput() { 
        return currentSpeed * 100.0; 
    }
    
    public boolean isAtBottom() { 
        try {
            return !bottomLimitSwitch.get(); // INVERTED: true when switch is pressed
        } catch (Exception e) {
            System.err.println("Error reading bottom limit: " + e.getMessage());
            return false;
        }
    }
    
    public boolean isAtTop() { 
        try {
            return !topLimitSwitch.get(); // INVERTED: true when switch is pressed
        } catch (Exception e) {
            System.err.println("Error reading top limit: " + e.getMessage());
            return false;
        }
    }
    
    // Method to get raw limit switch values (for debugging)
    public boolean getBottomLimitRaw() {
        try {
            return bottomLimitSwitch.get();
        } catch (Exception e) {
            System.err.println("Error reading bottom limit raw: " + e.getMessage());
            return false;
        }
    }
    
    public boolean getTopLimitRaw() {
        try {
            return topLimitSwitch.get();
        } catch (Exception e) {
            System.err.println("Error reading top limit raw: " + e.getMessage());
            return false;
        }
    }
    
    public String getStatus() { 
        return status; 
    }
    
    public boolean isMotorInitialized() {
        return motorInitialized;
    }
    
    @Override
    public void periodic() {
        double current = getCurrent();
        
        // Log elevator state periodically for debugging
        if (motorInitialized && (currentSpeed != 0 || current > 0.1)) {
            System.out.println("Elevator State - Speed: " + currentSpeed + 
                              ", Current: " + current + "A" +
                              ", Max Current: " + maxCurrentSeen + "A" +
                              ", Bottom Limit: " + isAtBottom() + " (raw: " + getBottomLimitRaw() + ")" +
                              ", Top Limit: " + isAtTop() + " (raw: " + getTopLimitRaw() + ")");
        }
        
        // Safety override - emergency stop if current too high
        if (current > 40.0) { // 40A is very high - likely stalled
            System.err.println("!!! CRITICAL: Elevator current exceeded 40A - EMERGENCY STOP !!!");
            stop();
            status = "EMERGENCY STOP - OVERCURRENT";
        } else if (current > 30.0) {
            System.err.println("!!! WARNING: Elevator current exceeded 30A - motor may be stalled !!!");
        }
    }
}