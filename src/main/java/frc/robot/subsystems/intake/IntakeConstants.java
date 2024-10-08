package frc.robot.subsystems.intake;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public interface IntakeConstants {
    int MOTOR_ID = 11;
    int LIMIT_SWITCH_ID = 8;
    double PEAK_CURRENT = 40.0; //TODO: find Value
    NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
    InvertedValue INVERTED = InvertedValue.Clockwise_Positive; //check the correct direction
    double COLLECT_CURRENT = 20; //TODO: find Value
    double FEED_INTAKE_CURRENT = 0.7; //TODO: find Value
    double FEED_WAIT_TIME = 0.2; //TODO: find Value
}
