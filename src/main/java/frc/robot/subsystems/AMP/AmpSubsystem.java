package frc.robot.subsystems.AMP;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class AmpSubsystem extends SubsystemBase implements AmpConstants{
    // Motor controller for the arm
    private final CANSparkMax _armMotor;
    
    // Encoder for position feedback
    private final TalonSRX encoder;

    //Singleton
    private static AmpSubsystem instance;
    public static AmpSubsystem getInstance(){
      if (instance == null){
        instance = new AmpSubsystem();
      }
      return instance;
    }
    

    private AmpSubsystem(){
        _armMotor = new CANSparkMax(MOTOR_ID, MotorType.kBrushless);
        encoder = new TalonSRX(ENCODER_ID);
        configs();
        resetToAbsultPosition();
    }

    // Method to move the arm to a specified position using Motion Magic
    public Command moveToPosition(double targetPosition) {
        
        // Clamp the target position to ensure it's within the limits
        double clampedTargetPosition = Math.max(MIN_POSITION, Math.min(MAX_POSITION, targetPosition));

        return this.runOnce(()->
            _armMotor.getPIDController().setReference(clampedTargetPosition, ControlType.kPosition));
    }

    // Method to return the current position of the arm
    public double getPosition() {
        return _armMotor.getEncoder().getPosition();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // You could add dashboard output or checks here if needed
    }

    private void configs(){
        SparkPIDController sparkPIDController = _armMotor.getPIDController();

        _armMotor.setInverted(INVERTED);
        _armMotor.setSmartCurrentLimit(CURRENT_LIMIT);
        _armMotor.setIdleMode(IDLE_MODE);

        //PID
        
        sparkPIDController.setP(kP);
        sparkPIDController.setD(kD);

        // forward and backward limits 
        
        _armMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, MAX_POSITION);
        _armMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, MIN_POSITION);

        _armMotor.getEncoder().setPositionConversionFactor(1/147);

        encoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    }

    public void resetToAbsultPosition(){
        _armMotor.getEncoder().setPosition(encoder.getSelectedSensorPosition());
    }
}