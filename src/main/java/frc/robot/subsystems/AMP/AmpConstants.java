package frc.robot.subsystems.AMP;

import com.revrobotics.CANSparkMax;

public interface AmpConstants {
    int MOTOR_ID = 0;
    int ENCODER_ID = 0;

    boolean INVERTED = false;
    CANSparkMax.IdleMode IDLE_MODE = CANSparkMax.IdleMode.kBrake;

    double GEAR_RATIO = 1 / 70;
    double TICKS_PER_DEGREE =  GEAR_RATIO / 360;

    // PID gains
    double kP = 0.1;
    double kI = 0.0;
    double kD = 0.0;

    // Feedforward (ks, kv) for controlling the arm movement
    double kS = 0.2;  // Static friction gain
    double kV = 0.1;  // Velocity gain

    int MIN_POSITION = -2;
    int MAX_POSITION = 100;

    // Voltage limit
    int CURRENT_LIMIT = 40;  // Max voltage

    // Motion Magic settings
    int CRUISE_VELOCITY = 1500;  // Adjust based on testing
    int ACCELERATION = 600;     // Adjust based on testing
}
