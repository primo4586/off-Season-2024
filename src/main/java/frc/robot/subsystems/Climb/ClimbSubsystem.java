// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

     /*
     create new m_motorLeft,m_motorRight
     *
     * @param CLIMBING_MOTOR_LEFT_ID : id of the left motor
     * @param CLIMBING_MOTOR_RIGHT_ID : id of right motor
     * 
     * */
    m_motorLeft = new CANSparkMax(CLIMBING_MOTOR_LEFT_ID, MotorType.kBrushless);
    m_motorRight = new CANSparkMax(CLIMBING_MOTOR_RIGHT_ID, MotorType.kBrushless);
    configs();

  }
  /*
   * command - set the same speed to both motors
   * @param speedMotorRight.getAsDouble() : the speed to set to motor 
   */
    public Command setSpeedCommand(DoubleSupplier speedMotorRight){
    return run(() -> {m_motorLeft.set(speedMotorRight.getAsDouble());
    m_motorRight.set(speedMotorRight.getAsDouble());});
 
  }
 

  /*
   * command - set different speed to motors
   * @param speedMotorLeft.getAsDouble() set this speed to motor left
   * @param speedMotorRight.getAsDouble() set this speed to motor right
   */
  public Command setSpeedCommand(DoubleSupplier speedMotorLeft, DoubleSupplier speedMotorRight ){
    return run(() -> {m_motorLeft.set(speedMotorLeft.getAsDouble());
    m_motorRight.set(speedMotorRight.getAsDouble());});
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("climb left motor position", m_motorLeft.getEncoder().getPosition());
    SmartDashboard.putNumber("climb right motor position", m_motorRight.getEncoder().getPosition());

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
