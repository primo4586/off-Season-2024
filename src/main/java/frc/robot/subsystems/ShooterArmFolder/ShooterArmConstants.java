package frc.robot.subsystems.ShooterArmFolder;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

import frc.robot.util.exterpolation.ExterpolationMap;


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

    double KP = 2;
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
    double MINIMUM_ERROR = 14;
    double RESET_SPEED = -0.2;

    double BASE_ANGLE = 35.5;
    double Pass_ANGLE = 50; //TODO: find value

    double RESET_SHOOTER_TIME_LIMIT = 10;
    double MEDIUM_SHOOTER_ANGEL = 50;

    double AMP_ANGLE = 40;

    ExterpolationMap SPEAKER_ANGLE_EXTERPOLATION = new ExterpolationMap()
            
            .put(1, 40.5)
            .put(1.45, 53.429)
            .put(1.768, 52.820)
            .put(2.358, 66.69)
            .put(2.64, 71.88)
            .put(2.98, 76.448)
            .put(3.68, 79.413);

}