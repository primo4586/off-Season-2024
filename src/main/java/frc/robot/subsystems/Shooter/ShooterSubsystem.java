// melon i have no idea wat im doing i tink i removed somthin idk
package frc.robot.subsystems.Shooter;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.apriltag.AprilTagDetector.Config;
import edu.wpi.first.wpilibj.DigitalInput;
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

 /**
   * Get the velocity of the Up motor atm
   */
  public double getDownMotorSpeed(){
    return down_motor.getVelocity().getValue();
  }
  
  /**
   * Get the velocity of the Up motor atm
   */
  public double getUpMotorSpeed(){
    return up_motor.getVelocity().getValue();
  }



  private void Configs(){
    
  }

 
}
