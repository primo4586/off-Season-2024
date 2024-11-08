package frc.robot.SmartDashbored;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Climb.ClimbSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.ShooterArmFolder.ShooterArmSubsystem;
import frc.robot.subsystems.Vision.AprilTagCamera;
import frc.robot.subsystems.Vision.Vision_Constants;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class NetworkTabels {
    //subsystems
    private static IntakeSubsystem intake = IntakeSubsystem.getInstance();
    private static ShooterArmSubsystem shooterArm = ShooterArmSubsystem.getInstance();
    private static ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    private static ClimbSubsystem climb = ClimbSubsystem.getInstance();
    private AprilTagCamera leftAprilTagCamera = new AprilTagCamera(Vision_Constants.K_RIGHT_CAMERA_NAME);

    // clock 
    private static double _clock;
    private static double _countDown;


    public static void updateValues(){
        clock();
        intake();
    }

    //clock func
    public static void setClock(double clock, double countDown){
        _clock = clock;
        _countDown = countDown;
    }
    public static void clock(){
        _clock -= _countDown;
        SmartDashboard.putNumber("clock", _clock);
    }

    //intake func
    public static void intake(){
        SmartDashboard.putBoolean("Note switch", intake.getSwitch());
    }

    //shooter func



}
