// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

public class ShooterarmSubsystem extands SubsystemBase implements ShooterarmSubsystem{
  private TalonFX m_shooterArmMotor;
  private DigitalInput m_limitSwitch;
  


private void configs(){
    TalonFXConfiguration configuration = new TalonFXConfiguration();
    MotionMagicConfigs mm =new MotionMagicConfigs();

    //peeks
    configuration.CurrentLimits.SupplyCurrentLimitEnable=true;
    configuration.CurrentLimits.SupplyCurrentLimit= PEAK_CURRENT;

    mm.MotionMagicCruiseVelocity=MM_CRUISE;
    mm.MotionMagicAcceleration=MM_ACCELERATION;
    mm.MotionMagicJerk=MM_JERK;

    //slot
    configuration.Slot0.kP=KP;
    configuration.Slot0.kD=KD;
    configuration.Slot0.kV=KV;
    configuration.Slot0.kS=KS;
    configuration.Slot0.kA=KA;

    // forward  
    configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable=true;
    configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable=true;
    configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold=FOWORD_LIMIT;
    configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold=BACKWARD_LIMIT;

    configuration.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable=true;
    configuration.HardwareLimitSwitch.ReverseLimitAutosetPositionValue=RESET_POSITION;

    //setings
    configuration.Feedback.SensorToMechanismRatio=TICKS_PER_DEGREE;
    configuration.MotorOutput.NeutralMode=NEUTRAL_MODE;
    configuration.MotorOutput.Inverted=INVERTED;

    StatusCode statusCode = StatusCode.StatusCodeNotInitialized;
    for(int i=0; i<5;i++){
      statusCode = m_shooterArmMotor.getConfigurator().apply(configuration);
      if(statusCode.isOK())
        break;



    }
    if(!statusCode.isOK())
    System.out.println("shooter Arm could not apply config, error cod"+ statusCode.toString());


  }
}

  
