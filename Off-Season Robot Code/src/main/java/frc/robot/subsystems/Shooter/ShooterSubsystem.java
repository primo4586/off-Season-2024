// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase implements ShooterConstants {
  /** Creates a new ShooterSubsystem. */
  private TalonFX _upMotor;
  private TalonFX _downMotor;
  private double targetSpeed = 0;

  /** using a singleton */
  public static ShooterSubsystem instance;

  public static ShooterSubsystem getInstance() {
    if (instance == null) {
      instance = new ShooterSubsystem();
    }
    return instance;
  }

  /** finding the motors mased on the id and can-bus name */
  public ShooterSubsystem() {
    _upMotor = new TalonFX(UPMOTOR_ID, Constants.CAN_BUS_NAME);
    _downMotor = new TalonFX(DOWNMOTOR_ID, Constants.CAN_BUS_NAME);
  }

  /** a command for setting a speed to the motors */
  public Command setShooterSpeed(double speed) {
    targetSpeed = speed;
    return runOnce(() -> {
      _upMotor.set(speed);
      _downMotor.set(speed);
    });
  }

  /** a command for stopping the motors */
  public Command stopMotor() {
    return runOnce(() -> {
      _upMotor.stopMotor();
      ;
      _downMotor.stopMotor();
      ;
    });
  }

  /** a command for checking if the motors are at the speed */
  public boolean isAtVelocity() {
    return Math.abs(_upMotor.getVelocity().getValue() - targetSpeed) < MINIMUM_ERROR
        && Math.abs(_downMotor.getVelocity().getValue() - targetSpeed) < MINIMUM_ERROR;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
