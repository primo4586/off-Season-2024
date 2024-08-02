// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class intakeSubsystem extends SubsystemBase implements intakeConstants{
  /** Creates a new intakeSubsystem. */
  private TalonFX _motor;
  private DigitalInput _limitSwitch;

  private static intakeSubsystem instance;
  public static intakeSubsystem getInstance(){
    if (instance == null) {
      instance = new intakeSubsystem();
    }
    return instance;
  }
  //im coping yair
  private intakeSubsystem() {
    _motor = new TalonFX(MOTOR_ID, Constants.CAN_BUS_NAME);
    _limitSwitch = new DigitalInput(LIMITSWITCH_ID);
    configs();
  }
  //were setting the motors speed like yair
  public Boolean getSwitch(){
    return _limitSwitch.get();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  private void configs(){}
}
