// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase implements ClimbConstants {
  private CANSparkMax m_motorRight;
  private CANSparkMax m_motorLeft;

  //instance:
  private static ClimbSubsystem instance;
  public static ClimbSubsystem getInstance() {
    if (instance == null) {
      instance = new ClimbSubsystem();
    }
    return instance;
  }

  /** Creates a new ClimbSubsystem. */
  private ClimbSubsystem() {
    configs();

     // This method will be called once per scheduler run
     //create new m_motorLeft,m_motorRight
    m_motorLeft = new CANSparkMax(CLIMBING_MOTOR_LEFT_ID, MotorType.kBrushless);
    m_motorRight = new CANSparkMax(CLIMBING_MOTOR_RIGHT_ID, MotorType.kBrushless);

  }
  //command - set the same speed to both motors
    public Command setSpeedCommand(DoubleSupplier speedMotorRight){
    return run(() -> {m_motorLeft.set(speedMotorRight.getAsDouble());
    m_motorRight.set(speedMotorRight.getAsDouble());});
 
  }

  //command - set different speed to motors
  public Command setSpeedCommand(DoubleSupplier speedMotorLeft, DoubleSupplier speedMotorRight ){
    return run(() -> {m_motorLeft.set(speedMotorLeft.getAsDouble());
    m_motorRight.set(speedMotorRight.getAsDouble());});
    
  }

  @Override
  public void periodic() {

  }

  private void configs(){
    m_motorRight.setInverted(false);
    m_motorLeft.setInverted(true);

    //current limit:
    m_motorRight.setSmartCurrentLimit(ClimbConstants.CURRENT_LIMIT);
    m_motorLeft.setSmartCurrentLimit(ClimbConstants.CURRENT_LIMIT);

    //idle mode:
    m_motorRight.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_motorLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);

  }
}
