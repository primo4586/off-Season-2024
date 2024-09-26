package frc.robot.subsystems.Shooter;

public interface ShooterConstants {
    // Shooter Motor IDs
    int UP_MOTOR_ID = 32; 
    int DOWN_MOTOR_ID = 33; 

    double MINIMUM_ERROR = 5;
    double YEET_CURRENT = 20; //find value

    // upper motor constants
    int UP_MOTOR_CURRENT_LIMIT = 40;
    int UP_MOTOR_CURRENT_THREASHOLD = 50;
    double UP_MOTOR_TIME_THREASHOLD = 0.1;

    // lowwer motor constants
    int DOWN_MOTOR_CURRENT_LIMIT = 40;
    int DOWN_MOTOR_CURRENT_THREASHOLD = 50;
    double DOWN_MOTOR_TIME_THREASHOLD = 0.1;
 
    // Motion Magic Values
    int MOTION_MAGIC_CRUISE_VELOCITY = 80;
    int MOTION_MAGIC_ACCELERATION = 160;
    int MOTION_MAGIC_JERK = 1600;

    double PEAK_FORWARD_VOLTAGE = 11.5;
    double PEAK_REVERSE_VOLTAGE = -11.5;

    double GEAR_RATIO = 0.5;

    int MAX_ERROR = 2;

    // PID values for up motor
    double UP_KP = 0.2;
    double UP_KD = 0;
    double UP_KS = 0.4586;
    double UP_KV = 0.064518;
    double UP_KA = 0.24784;

   // PID values for down motor
    double DOWN_KP = 0.022;
    double DOWN_KD = 0.0;
    double DOWN_KS = 0.2998046875;
    double DOWN_KV = 0.0583;
    double DOWN_KA = 0.079175;

   // Interpolatio
    double SHOOT_BASE_SPEED = 50;
    double SHOOT_STAGE_SPEED = 70;
    double SHOOT_SPEED = 70;
    double AMP_SPEED = 23;
    
    // Speed Values
    double BASE_SPEED = 50; //TODO: find value 

}
