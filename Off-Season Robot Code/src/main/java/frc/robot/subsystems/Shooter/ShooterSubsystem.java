// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase implements ShooterConstants{
  /** Creates a new ShooterSubsystem. */
  private TalonFX _upMotor;
  private TalonFX _downMotor;

  public static ShooterSubsystem instance;
  public static ShooterSubsystem getInstance() {
    if (instance == null) {
      instance = new ShooterSubsystem();
    }
    return instance;
  }

  public ShooterSubsystem() {
    _upMotor = new TalonFX(UPMOTOR_ID, Constants.CAN_BUS_NAME);
    _downMotor = new TalonFX(DOWNMOTOR_ID, Constants.CAN_BUS_NAME);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
