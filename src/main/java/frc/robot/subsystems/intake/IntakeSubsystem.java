// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase implements IntakeConstants{
  private TalonFX _motor;
  private DigitalInput _limitSwitch;
  private TorqueCurrentFOC currentFOC = new TorqueCurrentFOC(0);

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

  /**
   * returns the state of the switch
   * 
   */
  public boolean getSwitchCommand(){
    return _limitSwitch.get();
  }

  /**
   * sets the current to 20 amp
   * 
   */
  public Command setSpeedCommand(){
    return runOnce(() -> _motor.setControl(currentFOC.withOutput(COLLECT_CURRENT)));
  }

  public Command setSpeedCommand(double speed){
    return runOnce(() -> _motor.setControl(currentFOC.withOutput(speed)));
  }

  /**
   * Stops the motor
   * 
   */
  public Command stopIntakeCommand(){
    return runOnce(() -> _motor.stopMotor());
  }

  public Command coolectUntilNoteCommand(){
    return startEnd(() -> setSpeedCommand(),() -> stopIntakeCommand()).until(() -> getSwitchCommand());
  }

  public Command feedShooterCommand(){
    return startEnd(() -> _motor.setControl(currentFOC.withOutput(FEED_INTAKE_SPEED)),() -> stopIntakeCommand()).withTimeout(FEED_WAIT_TIME)
    .andThen(stopIntakeCommand());
  }

  public Command feedShooterCommand(double speed){
    return startEnd(() -> _motor.setControl(currentFOC.withOutput(speed)),() -> stopIntakeCommand()).withTimeout(FEED_WAIT_TIME)
    .andThen(stopIntakeCommand());
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
