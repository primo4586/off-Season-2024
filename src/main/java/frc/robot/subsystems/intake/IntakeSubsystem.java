// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase implements IntakeConstants{
  private TalonFX _motor;
  private DigitalInput _limitSwitch;
  //singelton
  private static IntakeSubsystem instance;
  public static IntakeSubsystem getInstance(){
    if (instance == null){
      instance = new IntakeSubsystem();
    }
    return instance;
  }
  /** Creates a new IntakeSubsystem. */
  private IntakeSubsystem() {
    _motor = new TalonFX(MOTOR_ID);
    _limitSwitch = new DigitalInput(LIMIT_SWITCH_ID);
    configs();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  private void configs(){
    // create the full MotionMagic
    TalonFXConfiguration configuration = new TalonFXConfiguration();

    //Peeks
    configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    configuration.CurrentLimits.SupplyCurrentLimit = PEAK_CURRENT;

    //settings
    configuration.MotorOutput.NeutralMode = NEUTRAL_MODE;
    configuration.MotorOutput.Inverted = INVERTED;

    //upload configs motor
    StatusCode statusCode = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++){
      statusCode = _motor.getConfigurator().apply(configuration);
      if(statusCode.isOK())
      break;
    }
    if (!statusCode.isOK())
    System.out.println("Intake could not apply config, error code:" + statusCode.toString());

  }
}
