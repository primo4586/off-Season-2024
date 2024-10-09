package frc.robot.subsystems.AMP;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

public class AmpSubsystem extends SubsystemBase implements AmpConstants{
    // Motor controller for the arm
    private final CANSparkMax _armMotor;
    
    // Encoder for position feedback
    private final CANcoder encoder;

    // Talon SRX for handling the encoder input
    private final Talon encoderTalon;

    private static AmpSubsystem instance;
    public static AmpSubsystem getInstance(){
      if (instance == null){
        instance = new AmpSubsystem();
      }
      return instance;
    }
    

    private AmpSubsystem() {
        // Initialize Spark MAX motor controller
        _armMotor = new CANSparkMax(MOTOR_ID, MotorType.kBrushless);  // ID 1
         encoderTalon = new Talon(ENCODER_ID);  // ID 2 for the Talon SRX controlling the encoder

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
        _armMotor.restoreFactoryDefaults();
        _armMotor.setIdleMode(IdleMode.kBrake);
        _armMotor.setSmartCurrentLimit(40);  // Limit current to prevent overdraw

        // Set voltage limits to avoid overdriving the motor
        _armMotor.enableVoltageCompensation(VOLTAGE_LIMIT);

        // Initialize the encoder with Talon SRX
     MotionMagicConfigs mm = new MotionMagicConfigs();

        // Set Motion Magic parameters (cruise velocity and acceleration)
        mm.configMotionCruiseVelocity = 
        encoderTalon.configMotionAcceleration(ACCELERATION);

        // Set PID gains for Motion Magic
        encoderTalon.config_kP(0, kP);
        encoderTalon.config_kI(0, kI);
        encoderTalon.config_kD(0, kD);

        // Set sensor phase depending on installation
        encoderTalon.setSensorPhase(true); // Adjust if necessary

        // Set NeutralMode to brake for holding position
        encoderTalon.setNeutralMode(NeutralModeValue.Brake);

        // Limit forward and reverse positions for safety
        encoderTalon.configForwardSoftLimitEnable(true);
        encoderTalon.configForwardSoftLimitThreshold(MAX_POSITION);
        encoderTalon.configReverseSoftLimitEnable(true);
        encoderTalon.configReverseSoftLimitThreshold(MIN_POSITION);
    }
}