package frc.robot.subsystems.intake;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public interface IntakeConstants {
    int MOTOR_ID = 21;
    int LIMIT_SWITCH_ID = 8;
    double PEAK_CURRENT = 40.0; //TODO: find Value
    NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
    InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive; //check the correct direction
    double COLLECT_CURRENT = 40; //TODO: find Value
    double FEED_INTAKE_CURRENT = 40; //TODO: find Value
    double FEED_WAIT_TIME = 0.5; //TODO: find Value
    double COLLECT_TIMEOUT = 10;
}
