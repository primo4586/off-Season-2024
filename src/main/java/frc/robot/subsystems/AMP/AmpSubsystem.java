package frc.robot.subsystems.AMP;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class AmpSubsystem extends SubsystemBase implements AmpConstants{
    // Motor controller for the arm
    private final CANSparkMax _armMotor;
    
    // Encoder for position feedback
    private final Talon encoder;

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
        encoder = new Talon(ENCODER_ID);
        _armMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        configs();
    }

    // Method to move the arm to a specified position using Motion Magic
    public void moveToPosition(double targetPosition) {
        // Clamp the target position to ensure it's within the limits
        targetPosition = Math.max(MIN_POSITION, Math.min(MAX_POSITION, targetPosition));

        // Calculate the feedforward output based on velocity
        double feedforwardOutput = feedforward.calculate(encoderTalon.getSelectedSensorVelocity());

        // Use Motion Magic to move to the target position
    }

    // Method to return the current position of the arm
    public double getPosition() {
        return encoderTalon.getSelectedSensorPosition();
    }

    // Method to return the current velocity of the arm
    public double getVelocity() {
        return encoderTalon.getSelectedSensorVelocity();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // You could add dashboard output or checks here if needed
    }

    private void configs(){
        SparkPIDController configuration = _armMotor.getPIDController();
        MotionMagicConfigs mm = new MotionMagicConfigs();
         
        //motion magic
        mm.MotionMagicCruiseVelocity = CRUISE_VELOCITY; 
        mm.MotionMagicAcceleration = ACCELERATION;

        _armMotor.setInverted(INVERTED);
        _armMotor.setSmartCurrentLimit(CURRENT_LIMIT);
        _armMotor.setIdleMode(IDLE_MODE);

        //PID
        
        configuration.setP(kP);
        configuration.setI(kD);


        // forward and backward limits 
        
        _armMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, MAX_POSITION);
        _armMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, MIN_POSITION);

        
 


    }
}