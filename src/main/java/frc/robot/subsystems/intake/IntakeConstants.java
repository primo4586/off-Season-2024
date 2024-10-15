package frc.robot.subsystems.intake;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public interface IntakeConstants {
    int MOTOR_ID = 21;
    int LIMIT_SWITCH_ID = 8;
    double PEAK_VOLTAGE = 12.0; //TODO: find Value
    NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
    InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive; //check the correct direction
    double COLLECT_VOLTAGE = 6; //TODO: find Value
    double FEED_INTAKE_VOLTAGE = 10; //TODO: find Value
    double FEED_WAIT_TIME = 1; //TODO: find Value
    double COLLECT_TIMEOUT = 10;
    double PLAY_CURRNET = 20; //TODO find value
    

}
