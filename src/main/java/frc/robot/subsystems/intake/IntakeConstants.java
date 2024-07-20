package frc.robot.subsystems.intake;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public interface IntakeConstants {
    int MOTOR_ID = 0; //TODO: find ID
    int LIMIT_SWITCH_ID = 0;
    double PEAK_CURRENT = 40.0;
    NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
    InvertedValue INVERTED = InvertedValue.Clockwise_Positive; //check the correct direction
    double COLLECT_CURRENT = 20;
    double FEED_INTAKE_SPEED = 0.7;
    double FEED_WAIT_TIME = 0.2;
}
