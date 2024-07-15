// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooterarm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.ShooterConstants;

public class ShooterarmSubsystem extends SubsystemBase implements ShooterConstants{
  /** Creates a new ShooterarmSubsystem. */
  private TalonFX Motor;
  
  public ShooterarmSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
