// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class intakeSubsystem extends SubsystemBase implements intakeConstants {
  /** Creates a new intakeSubsystem. */
  private TalonFX _motor;
  private DigitalInput _limitSwitch;

  private static intakeSubsystem instance;

  public static intakeSubsystem getInstance() {
    if (instance == null) {
      instance = new intakeSubsystem();
    }
    return instance;
  }

  /** finding the motor and digitalInput based on the id and can-bus name */
  private intakeSubsystem() {
    _motor = new TalonFX(MOTOR_ID, Constants.CAN_BUS_NAME);
    _limitSwitch = new DigitalInput(LIMITSWITCH_ID);
    configs();
  }

  /** setting the motors speed */
  public Command setCurrentCommand(Double current) {
    return runOnce(() -> {
      _motor.set(current);
      ;
    });
  }

  /** running the intake motor until a note is detected */
  public Command collectUntilNoteCommand() {
    return runEnd(() -> setCurrentCommand(MOTOR_CURRENT), () -> stopMotorCommand()).until(() -> getSwitch());

  }

  /** getting the signal input of the limitSwitch */
  public Boolean getSwitch() {
    return _limitSwitch.get();
  }

  /** feeding the note to the Shooter */
  public Command feedShooterCommand() {
    return runOnce(() -> {
      _motor.set(FEED_SPEED);
      ;
    });
  }

  /** stopping the motor */
  public Command stopMotorCommand() {
    return runOnce(() -> {
      _motor.stopMotor();
      ;
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void configs() {
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
    motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
    motorConfigs.CurrentLimits.SupplyCurrentThreshold = CURRENT_THRESHOLD;
    motorConfigs.CurrentLimits.SupplyTimeThreshold = TIME_THRESHOLD;
    motorConfigs.MotorOutput.NeutralMode = NEUTRALMODE_VALUE;
    motorConfigs.MotorOutput.Inverted = MOTOR_DIRECTIONS;

    // upload configs to motor
    StatusCode statusCode = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++) {
      _motor.getConfigurator().apply(motorConfigs);
      if (statusCode.isOK())
        break;
    }
    if (!statusCode.isOK())
      System.out.println("Intake could not apply config, error code:" + statusCode.toString());

  }
}
