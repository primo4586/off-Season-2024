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
  private TalonFX up_Motor;
  private TalonFX down_Motor;
  private double targetSpeed = 0;

  // using a singleton
  private static ShooterSubsystem instance;
  /** if instance does not exist create one if it does just return it */
  public static ShooterSubsystem getInstance() {
    if (instance == null) {
      instance = new ShooterSubsystem();
    }
    return instance;
  }

  /** finding the motors mased on the id and can-bus name */
  private ShooterSubsystem() {
    up_Motor = new TalonFX(UP_MOTOR_ID, Constants.CAN_BUS_NAME);
    down_Motor = new TalonFX(DOWN_MOTOR_ID, Constants.CAN_BUS_NAME);
  }

  /** a command for setting a speed to the motors */
  public Command setShooterSpeed(double speed) {
    targetSpeed = speed;
    return runOnce(() -> {
      up_Motor.set(speed);
      down_Motor.set(speed);
    });
  }

  /** a command for stopping the motors */
  public Command stopMotor() {
    return runOnce(() -> {
      up_Motor.stopMotor();
      ;
      down_Motor.stopMotor();
      ;
    });
  }

  /** a command for checking if the motors are at the speed */
  public boolean isAtVelocity() {
    return Math.abs(up_Motor.getVelocity().getValue() - targetSpeed) < MINIMUM_ERROR
        && Math.abs(down_Motor.getVelocity().getValue() - targetSpeed) < MINIMUM_ERROR;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
