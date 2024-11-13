package frc.robot.SmartDashbored;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Misc;
import frc.robot.Telemetry;
import frc.robot.subsystems.Climb.ClimbSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.ShooterArmFolder.ShooterArmSubsystem;
import frc.robot.subsystems.Vision.AprilTagCamera;
import frc.robot.subsystems.Vision.ObjectDetectionCamera;
import frc.robot.subsystems.Vision.Vision_Constants;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class NetworkTabels {
    //subsystems
    private static final IntakeSubsystem intake = IntakeSubsystem.getInstance();
    private static final ShooterArmSubsystem shooterArm = ShooterArmSubsystem.getInstance();
    private static final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    private static final ClimbSubsystem climb = ClimbSubsystem.getInstance();

    // cameras 
    private static final AprilTagCamera leftAprilTagCamera = Misc.LEFT_CAMERA;
    private static final AprilTagCamera rightAprilTagCamera = Misc.RIGHT_CAMERA;
    private static final ObjectDetectionCamera noteCamera = Misc.NOTE_CAMERA;

    // swerve 



    // clock 
    private static double _clock;
    private static double _countDown;

    // field 
    private static final Field2d m_field = new Field2d();


    public static void updateValues(){
        updateClock();
        updateIntake();
        updateVision();
        updateRobotPose();
    }

    //clock func
    public static void setClock(double clock, double countDown){
        _clock = clock;
        _countDown = countDown;
    }
    public static void updateClock(){
        _clock -= _countDown;
        SmartDashboard.putNumber("clock", _clock);
        if (_clock <= 0){setClock(0, 0);}
    }

    //intake func
    public static void updateIntake(){
        SmartDashboard.putBoolean("Note switch", intake.getSwitch());
    }

    // field func
    public static void setField(){SmartDashboard.putData("Field", m_field);}

    public static void updateRobotPose(){
        m_field.setRobotPose(Misc.m_Telemetry.getRobotPose());}

    //vision
    public static void updateVision(){
        SmartDashboard.putBoolean("April tag detection", leftAprilTagCamera.seeTarget() || rightAprilTagCamera.seeTarget());
    }



}
