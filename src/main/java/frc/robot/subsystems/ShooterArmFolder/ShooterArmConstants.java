package frc.robot.subsystems.ShooterArmFolder;

import com.ctre.phoenix6.controls.MotionMagicVoltage;


public interface ShooterArmConstants {
    //Technical Constants
    int SHOOTER_ARM_ID = 31;
    int SWITCH_ID = 6;
    int ENCODER_COUNTS_PER_REVOLUTION = 1;
    double GEAR_RATIO = 100 / 22 * 100;
    double TICKS_PER_DEGREE = ENCODER_COUNTS_PER_REVOLUTION * GEAR_RATIO / 360.0;

    // MotionMagic Constants
    double MM_CRUISE = 80;
    double MM_ACCELERATION = 300;
    double MM_JERK = 1600;

    double KP = 0.35;
    double KD = 0;
    double KS  = 0;
    double KA = 0;
    double KV = 0;

    double PEAK_CURRENT = 50;

    double FOWORD_LIMIT = 80; //TODO find value
    double BACKWARD_LIMIT = -1;//TODO find value

    MotionMagicVoltage MOTION_MAGIC_VOLTAGE = new MotionMagicVoltage(0,
    true,
    0.0,
    0,
    true,
    true,
    true);

    // Condition Constants
    double MINIMUM_ERROR = 0.1;
    double RESET_SPEED = -0.2;

    double BASE_ANGLE = 60; //TODO: find value

    double RESET_SHOOTER_TIME_LIMIT = 10;
    double MEDIUM_SHOOTER_ANGEL = 45; //TODO find value

}