package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Manipulator extends SubsystemBase {
    private final SparkMax motor = new SparkMax(Constants.CAN_ID_MANIPULATOR, MotorType.kBrushless);
    private double currentSpeed = 0.0;
    private String status = "OFF";
    
    public void setSpeed(double input) {
        if (Math.abs(input) < Constants.DEADBAND) {
            currentSpeed = 0.0;
            status = "OFF";
        } else if (input < 0) {
            currentSpeed = -input * Constants.INTAKE_SPEED;
            status = "INTAKE";
        } else {
            currentSpeed = input * Constants.OUTPUT_SPEED;
            status = "OUTPUT";
        }
        
        motor.set(currentSpeed);
    }
    
    public void stop() { 
        setSpeed(0); 
    }
    
    public double getCurrent() { 
        return motor.getOutputCurrent(); 
    }
    
    public String getStatus() { 
        return status; 
    }
    
    @Override
    public void periodic() {}
}