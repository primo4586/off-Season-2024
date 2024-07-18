// melon i have no idea wat im doing i tink i removed somthin idk
package frc.robot.subsystems.Shooter;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.apriltag.AprilTagDetector.Config;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class ShooterSubsystem extends SubsystemBase implements ShooterConstants {
  private TalonFX up_motor;
  private TalonFX down_motor;
  private static final MotionMagicVelocityTorqueCurrentFOC mm = new MotionMagicVelocityTorqueCurrentFOC(0,MOTION_MAGIC_ACCELERATION,true,0.0,0,false,false,false);

  // singleton  
  private static ShooterSubsystem instance;
  public static ShooterSubsystem getInstance(){
    if(instance==null) {
      instance = new ShooterSubsystem();
    }
    return instance;
  }
  private ShooterSubsystem() {
    up_motor = new TalonFX(UP_MOTOR_ID, Constants.CAN_BUS_NAME);
    down_motor = new TalonFX(DOWN_MOTOR_ID, Constants.CAN_BUS_NAME);
    Configs();
  }

  /*** A command to stop all the motors */
  public Command stopMotors(){
    return runOnce(() -> {
      up_motor.stopMotor();
      down_motor.stopMotor();
    });
  }

  public Command setShooterspeed(double speed){
    return runOnce(() -> {
      up_motor.setControl(mm.withVelocity(speed));
      down_motor.setControl(mm.withAcceleration(speed));

    });
  }



  private void Configs(){
    // declaring Configs
    TalonFXConfiguration upConfigs = new TalonFXConfiguration();
    TalonFXConfiguration downConfigs = new TalonFXConfiguration();
    MotionMagicConfigs shooterMM = new MotionMagicConfigs();

    // motion magic
    shooterMM.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
    upConfigs.MotionMagic = shooterMM;
    downConfigs.MotionMagic = shooterMM;

    // motors limitations
    upConfigs.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
    downConfigs.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
    


  }

 
}
