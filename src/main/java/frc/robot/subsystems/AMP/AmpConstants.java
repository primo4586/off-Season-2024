package frc.robot.subsystems.AMP;

public interface AmpConstants {
    int MOTOR_ID = 0;
    int ENCODER_ID = 0;
    // PID gains
    double kP = 0.1;
    double kI = 0.0;
    double kD = 0.0;

    // Feedforward (ks, kv) for controlling the arm movement
    double ks = 0.2;  // Static friction gain
    double kv = 0.1;  // Velocity gain

    double MIN_POSITION = -2.0;
    double MAX_POSITION = 100.0;

    // Voltage limit
    double VOLTAGE_LIMIT = 10.0;  // Max voltage

    // Motion Magic settings
    int CRUISE_VELOCITY = 1500;  // Adjust based on testing
    int ACCELERATION = 600;     // Adjust based on testing
}
