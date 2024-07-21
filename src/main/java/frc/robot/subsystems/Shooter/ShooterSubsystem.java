// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase implements ShooterConstants {
  /** Creates a new ShooterSubsystem. */
  private TalonFX up_Motor;
  private TalonFX down_Motor;
  private static final MotionMagicVelocityTorqueCurrentFOC mm = new MotionMagicVelocityTorqueCurrentFOC(0,
      MOTION_MAGIC_ACCELERATION, true, 0.0, 0, false, false, false);

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
    configs();
  }

  /** a command for setting a speed to the motors */
  public Command setShooterSpeed(double speed) {
    return runOnce(() -> {
      up_Motor.setControl(mm.withVelocity(speed));
      down_Motor.setControl(mm.withVelocity(speed));
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

  /** a command for shooting from base */
  public Command shootFromBase() {
    return runOnce(() -> {
      setShooterSpeed(SHOOT_BASE_SPEED);
      ;
    });
  }

  /** a command for shooting from Far */
  public Command shootFromFar() {
    return runOnce(() -> {
      setShooterSpeed(SHOOT_STAGE_SPEED);
      ;
    });
  }

  /** a command for checking if the motors are at the speed */
  public boolean isAtVelocity() {
    return Math.abs(up_Motor.getVelocity().getValue() - mm.Velocity) < MINIMUM_ERROR
        && Math.abs(down_Motor.getVelocity().getValue() - mm.Velocity) < MINIMUM_ERROR;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void configs() {
    // declaring Configs
    TalonFXConfiguration upConfigs = new TalonFXConfiguration();
    TalonFXConfiguration downConfigs = new TalonFXConfiguration();
    MotionMagicConfigs shooterMM = new MotionMagicConfigs();

    // giving motion magic values
    shooterMM.MotionMagicCruiseVelocity = MOTION_MAGIC_CRUISE_VELOCITY;
    shooterMM.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
    shooterMM.MotionMagicJerk = MOTION_MAGIC_JERK;
    upConfigs.MotionMagic = shooterMM;
    downConfigs.MotionMagic = shooterMM;

    // giving PID values
    upConfigs.Slot0.kP = UP_KP;
    upConfigs.Slot0.kD = UP_KD;
    upConfigs.Slot0.kS = UP_KS;
    upConfigs.Slot0.kV = UP_KV;
    upConfigs.Slot0.kA = UP_KA;

    downConfigs.Slot0.kP = DOWN_KP;
    downConfigs.Slot0.kD = DOWN_KD;
    downConfigs.Slot0.kS = DOWN_KS;
    downConfigs.Slot0.kV = DOWN_KV;
    downConfigs.Slot0.kA = DOWN_KA;

    // max voltage for m_shooterMotor
    upConfigs.Voltage.PeakForwardVoltage = PEAK_FORWARD_VOLTAGE;
    upConfigs.Voltage.PeakReverseVoltage = PEAK_REVERSE_VOLTAGE;

    downConfigs.Voltage.PeakForwardVoltage = PEAK_FORWARD_VOLTAGE;
    downConfigs.Voltage.PeakReverseVoltage = PEAK_REVERSE_VOLTAGE;

    upConfigs.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    downConfigs.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    upConfigs.CurrentLimits.SupplyCurrentLimit = UP_MOTOR_CURRENT_LIMIT;
    upConfigs.CurrentLimits.SupplyCurrentThreshold = UP_MOTOR_CURRENT_THREASHOLD;
    upConfigs.CurrentLimits.SupplyTimeThreshold = UP_MOTOR_TIME_THREASHOLD;
    upConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

    downConfigs.CurrentLimits.SupplyCurrentLimit = DOWN_MOTOR_CURRENT_LIMIT;
    downConfigs.CurrentLimits.SupplyCurrentThreshold = DOWN_MOTOR_CURRENT_THREASHOLD;
    downConfigs.CurrentLimits.SupplyTimeThreshold = DOWN_MOTOR_TIME_THREASHOLD;
    downConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

    up_Motor.setNeutralMode(NeutralModeValue.Coast);
    down_Motor.setNeutralMode(NeutralModeValue.Coast);

    upConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    downConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    /* Speed up signals for better characterization data */
    BaseStatusSignal.setUpdateFrequencyForAll(1000, up_Motor.getVelocity());
    BaseStatusSignal.setUpdateFrequencyForAll(1000, down_Motor.getVelocity());

    up_Motor.optimizeBusUtilization();
    down_Motor.optimizeBusUtilization();

    // Checking if up_Motor apply configs
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = up_Motor.getConfigurator().apply(upConfigs);
      if (status.isOK())
        break;
      System.out.println("status of upper motor configuration is not okay");
    }

  }

}
