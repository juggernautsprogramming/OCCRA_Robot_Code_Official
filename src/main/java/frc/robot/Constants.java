package frc.robot;

public final class Constants {
    // CAN IDs
    public static final int CAN_ID_LEFT_FRONT = 1;
    public static final int CAN_ID_LEFT_REAR = 2;
    public static final int CAN_ID_RIGHT_FRONT = 3;
    public static final int CAN_ID_RIGHT_REAR = 4;
    public static final int CAN_ID_ELEVATOR = 5;
    public static final int CAN_ID_MANIPULATOR = 6;
    
    // DIO Ports
    public static final int DIO_ELEVATOR_BOTTOM_LIMIT = 0;
    public static final int DIO_ELEVATOR_TOP_LIMIT = 1;
    
    // Mechanism Speeds
    public static final double ELEVATOR_MAX_SPEED = 0.5;
    public static final double INTAKE_SPEED = 1;
    public static final double OUTPUT_SPEED = -1;
    
    // Drive Constants
    public static final double SPEED_SCALE = 0.7;
    public static final double TURN_SCALE = 0.6;
    public static final double DEADBAND = 0.1;
    public static final double NUDGE_SPEED = 0.40;
    
    // Autonomous Constants
    public static final double TURN_TIME = 1.69;
    public static final double TURN_SPEED = 0.5;
}