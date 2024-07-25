// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import java.net.Socket;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.PubSub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  private CANSparkMax m_motorRight;
  private CANSparkMax m_motorLeft;

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
    m_motorLeft = new CANSparkMax(ClimbConstants.CLIMBING_MOTOR_LEFT_ID, MotorType.kBrushless);
    m_motorRight = new CANSparkMax(ClimbConstants.CLIMBING_MOTOR_RIGHT_ID, MotorType.kBrushless);

  }
    public Command setSpeedCommand(DoubleSupplier speedMotorRight){
    return run(() -> {m_motorLeft.set(speedMotorRight.getAsDouble());
    m_motorRight.set(speedMotorRight.getAsDouble());});
 
  }

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

    m_motorRight.setSmartCurrentLimit(ClimbConstants.CURRENT_LIMIT);
    m_motorLeft.setSmartCurrentLimit(ClimbConstants.CURRENT_LIMIT);

    m_motorRight.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_motorLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);



  }
}
