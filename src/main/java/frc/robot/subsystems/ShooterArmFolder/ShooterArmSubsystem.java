package frc.robot.subsystems.ShooterArmFolder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;

/**
 * This subsystem is resposible for the "line up shot"
 * @arthur Eilon.h
 * @Version 2.0.1
 */

 import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
 import com.ctre.phoenix6.configs.TalonFXConfiguration;
 import com.ctre.phoenix6.configs.TalonFXConfigurator;
 import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
 import com.ctre.phoenix6.signals.ForwardLimitValue;
 import com.ctre.phoenix6.signals.InvertedValue;
 import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
 import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 import edu.wpi.first.wpilibj2.command.Command;
 import edu.wpi.first.wpilibj2.command.Commands;
 import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import static edu.wpi.first.units.MutableMeasure.mutable;
 
 public class ShooterArmSubsystem extends SubsystemBase implements ShooterArmConstants{
   private TalonFX m_shooterArmMotor;
     // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.

  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.

   private final MotionMagicExpoTorqueCurrentFOC mm = new MotionMagicExpoTorqueCurrentFOC(0);
   private final DigitalInput m_limitSwitch = new DigitalInput(SWITCH_ID);
       private final VoltageOut m_sysIdControl = new VoltageOut(0);

       private final SysIdRoutine m_upSysIdRoutine =
       new SysIdRoutine(
           new SysIdRoutine.Config(
               null,         // Use default ramp rate (1 V/s)
               Volts.of(4), // Reduce dynamic voltage to 4 to prevent brownout
               null,          // Use default timeout (10 s)
                                      // Log state with Phoenix SignalLogger class
               (state)->{
                 double velocity = m_shooterArmMotor.getVelocity().getValueAsDouble();
                 double position = m_shooterArmMotor.getPosition().getValueAsDouble();
                 double appliedVoltage = m_shooterArmMotor.getMotorVoltage().getValueAsDouble();

                 SignalLogger.writeString("sysid_state",state.toString());
                 SignalLogger.writeDouble("fly_wheel velocity", velocity);
                 SignalLogger.writeDouble("fly_wheel position ", position);
                 SignalLogger.writeDouble("fly_wheel voltage", appliedVoltage);

               }),
           new SysIdRoutine.Mechanism(
               (Measure<Voltage> volts)-> m_shooterArmMotor.setControl(m_sysIdControl.withOutput(volts.in(Volts))),
               null,
               this));
   // singelton
   private static ShooterArmSubsystem instance;

   public static ShooterArmSubsystem getInstance(){
     if (instance == null)
       instance = new ShooterArmSubsystem();
     return instance;
   }
 
   /**
    * Constructor
    */
   private ShooterArmSubsystem() {
      
     m_shooterArmMotor = new TalonFX(SHOOTER_ARM_ID, Constants.CAN_BUS_NAME); // crearts new motor
     configs();
     sysidConfigs();
   }
 
    /**
     * Set the motor position
     * @param pose
     */
    public void setPosition(double pose) {
     m_shooterArmMotor.setPosition(pose);
   }
 
   /**
    * Gets the arm position
    * @return The posirion in degrees
    */
   public double getArmPose(){
     return m_shooterArmMotor.getPosition().getValue();
   }
 
   
   public boolean isArmReady(){
     return (Math.abs(getArmPose() - mm.Position) < MINIMUM_ERROR);
   }
 
   /**
    * Check if the switch is open
    * @return If the switch is open or not 
    */
   public boolean getSwitch(){
     return !(m_shooterArmMotor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround);
   }
 
   /**
    * Set the speed of the motor
    * @param speed Mhe wanted speed in double
    */
 
   public void coast() {
     m_shooterArmMotor.setNeutralMode(NeutralModeValue.Coast);
   }
 
   public void breakMode() {
     m_shooterArmMotor.setNeutralMode(NeutralModeValue.Brake);
   }
 
 
   /**
    * Reset the arm
    */
   public void manualZeroShooterArm() {
     coast();
     if (getSwitch()) {
       while (getSwitch()) {
       }
     }
     while (!getSwitch()) {
     }
     SmartDashboard.putBoolean("zerod out shooter", true);
     setPosition(0);
     breakMode();
   }
   public boolean getReverseLimit() {
     return !m_limitSwitch.get();
   }
   /**
    * Prepare the home command, if the reverse limit switch is pressed, do
    * nothing, otherwise move 
    the motor to the reverse limit switch position
    * at a high speed and wait for the switch to be pressepd
    * 
    * @return The command
    */
   public Command prepareHomeCommand() {
      return runEnd(() -> {m_shooterArmMotor.set(RESET_SPEED); System.out.println("switch not on");}
      ,() ->{ m_shooterArmMotor.stopMotor();
      setPosition(0);})
      .until(() -> getReverseLimit()).withTimeout(RESET_SHOOTER_TIME_LIMIT);
    }
    /* 
     return !getReverseLimit()
         ? Commands.none()
         : (runOnce(() -> m_shooterArmMotor.set(RESET_SPEED)).andThen(Commands.waitUntil(() -> !getReverseLimit())))
             .withTimeout(3);
     */
    public Command setSpeed(double speed){
      return runEnd(() -> m_shooterArmMotor.set(speed),
       () -> m_shooterArmMotor.stopMotor());
    }
   /**
    * Move the arm to a degree
    * @param degree
    * @return
    */
   public Command moveArmTo(double degree){
     return runOnce(() -> m_shooterArmMotor.setControl(mm.withPosition(degree)));
   }
   
  public Command moveArmToBase() {
    return runOnce(() -> m_shooterArmMotor.setControl(mm.withPosition(BASE_ANGLE)));
   }
   public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_upSysIdRoutine.quasistatic(direction);
}
public Command sysIdDynamic(SysIdRoutine.Direction direction) {
;
    return m_upSysIdRoutine.dynamic(direction);
}  
   
 
   @Override
   public void periodic() {
    SmartDashboard.putBoolean("shooter switch ", getReverseLimit());
    SmartDashboard.putNumber("shooter angel ", getArmPose());
     // This method will be called once per scheduler run
   }
 
   private void configs(){
     // create the full MotionMagic
     TalonFXConfiguration configuration = new TalonFXConfiguration();
     MotionMagicConfigs mm = new MotionMagicConfigs();
 
     mm.MotionMagicCruiseVelocity = MM_CRUISE;
     mm.MotionMagicAcceleration = MM_ACCELERATION;
     mm.MotionMagicJerk = MM_JERK;
 
     //Slot:
     configuration.Slot0.kP = KP;
     configuration.Slot0.kD = KD;
     configuration.Slot0.kV = KV;
     configuration.Slot0.kS = KS;
 
   //Peeks:
     configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
     configuration.CurrentLimits.SupplyCurrentLimit = PEAK_CURRENT;
 
 
     // forward and backward limits 
     configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
     configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
     configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = FOWORD_LIMIT;
     configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = BACKWARD_LIMIT;
     configuration.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
     configuration.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0;
     configuration.HardwareLimitSwitch.ReverseLimitEnable = true;
 
     configuration.Feedback.SensorToMechanismRatio = TICKS_PER_DEGREE; 
     configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
     configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
     StatusCode statusCode = StatusCode.StatusCodeNotInitialized;
 
 
     //upload configs to motor
     for (int i = 0; i < 5; i++){
       statusCode = m_shooterArmMotor.getConfigurator().apply(configuration);
       if (statusCode.isOK())
         break;
     }
     if (!statusCode.isOK())
       System.out.println("Shooter Arm could not apply config, error code:" + statusCode.toString());
   }
      /**
      * settings for sysid
      */
      private void sysidConfigs(){
        BaseStatusSignal.setUpdateFrequencyForAll(250,
        m_shooterArmMotor.getPosition(),
        m_shooterArmMotor.getVelocity(),
        m_shooterArmMotor.getMotorVoltage());

        /* Optimize out the other signals, since they're not useful for SysId */
        m_shooterArmMotor.optimizeBusUtilization();
      }
 }