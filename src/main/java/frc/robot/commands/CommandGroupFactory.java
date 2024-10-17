package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Climb.ClimbSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.ShooterArmFolder.ShooterArmSubsystem;
import frc.robot.subsystems.Vision.AprilTagCamera;
import frc.robot.subsystems.Vision.ObjectDetectionCamera;
import frc.robot.subsystems.Vision.Vision_Constants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.TunerConstants;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class CommandGroupFactory {
        private static double MaxSpeed =  1 * TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
        private static double MaxAngularRate = 3 * Math.PI;//1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

        private static final IntakeSubsystem intake = IntakeSubsystem.getInstance();
        private static final ClimbSubsystem climb = ClimbSubsystem.getInstance();
        private static final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
        private static final ShooterArmSubsystem shooterArm = ShooterArmSubsystem.getInstance();
        private static final CommandSwerveDrivetrain swerve = TunerConstants.DriveTrain;
        private static AprilTagCamera leftAprilTagCamera = new AprilTagCamera(Vision_Constants.K_LEFT_CAMERA_NAME);
        private static ObjectDetectionCamera noteCamera = new ObjectDetectionCamera(Vision_Constants.K_NOTE_CAMERA_NAME);

        private final static CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
        private final static CommandXboxController driverController = new CommandXboxController(0); // My joystick
        private final static SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-
        private static DoubleSupplier slowMode = () -> driverController.rightTrigger().getAsBoolean() ? 0.2 : 1;

        public static Command driveSwerve(){
                return drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * MaxSpeed * slowMode.getAsDouble()) // Drive forward with
                    // negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed *  slowMode.getAsDouble()) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate *  slowMode.getAsDouble()) // Drive counterclockwise with negative
                                                                                        // X (left)
                );
        }

        public static Command passNote(){
                return new ParallelDeadlineGroup(Commands.waitSeconds(0.02) //rio cycle
                .andThen(alignCommand(calculateAngleToPoint(Constants.passPosePoint))).andThen
                (Commands.waitUntil(() -> shooter.isAtVelocity() && shooterArm.isArmReady())).andThen(intake.feedShooterCommand()),
                shooter.shootFromFar(), shooterArm.moveArmToPass());
        }

        public static Command  alignToNote(){
                return noteCamera.getDetectingObject() ? alignCommand(new Rotation2d(noteCamera.getAngleFromTarget())) : Commands.none();
        }
        public static Command collectUntilNote() {
                // return new
                // SequentialCommandGroup(intake.coolectUntilNoteCommand(),intake.feedBack());
                return intake.setCurrentCommand()
                                .andThen(Commands.waitUntil(() -> intake.getSwitchCommand()))
                                .andThen(intake.feedBack());
        }

        public static Command shootFromBase() {
                return new ParallelDeadlineGroup(Commands.waitSeconds(0.02)// waits one rio scyle
                                .andThen(Commands.waitUntil(() -> shooter.isAtVelocity() && shooterArm.isArmReady())
                                                .andThen(intake.feedShooterCommand())),
                                shooterArm.moveArmToBase(), shooter.shootFromBase());
        }

        public static Command prepareToShoot() {// TODO: add shooter arm
                return new ParallelCommandGroup(shooter.shootFromFar(),
                shooterArm.speakerAngleExterapolateCommand(Constants.distanceFromSpeaker));
        }

        // TODO: add swerve steer
        public static Command shootFromMidum() {
                return new ParallelDeadlineGroup(Commands.waitSeconds(0.02)// waits one rio scyle
                                .andThen(Commands.waitUntil(() -> shooter.isAtVelocity() && shooterArm.isArmReady())
                                                .andThen(intake.feedShooterCommand())),
                                shooterArm.moveArmToMedium(), shooter.shootFromBase());
        }

        public static Command yeet() {
                return new ParallelDeadlineGroup(Commands.waitSeconds(2).andThen(intake.feedShooterCommand()),
                                shooter.shootFromFar());
        }

        public static Command Amp() {
                return new ParallelDeadlineGroup(Commands.waitSeconds(0.02)
                .andThen(Commands.waitUntil(() -> shooter.isAtVelocity() && shooterArm.isArmReady()))
                .andThen(Commands.waitSeconds(0.2)).andThen(intake.feedShooterCommand()),
                                shooter.setShooterSpeed(21.5), shooterArm.moveArmTo(40));
        }

        public static Command shotSpeakerCommand() {
                return new ParallelDeadlineGroup( leftAprilTagCamera.seeTarget() ? alignCommand(calculateAngleToPoint(Constants.speakerPosePoint)) :
                Commands.waitSeconds(0.02)// roi sycle 
                                
                                                .andThen(Commands.waitUntil(
                                                                () -> shooter.isAtVelocity() && shooterArm.isArmReady()).withTimeout(3)
                                                                .andThen(intake.feedShooterCommand())),

                                shooterArm.speakerAngleExterapolateCommand(Constants.distanceFromSpeaker),

                                shooter.shootFromFar());
        }

         public static Command alignCommand(Rotation2d target) {

                PIDController headingPid = new PIDController(0.155, 0.0, 0);
                headingPid.enableContinuousInput(-180, 180);
                headingPid.setTolerance(0.5);
                headingPid.setSetpoint(0);

                SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();

                return new RunCommand(
                                () -> swerve.setControl(
                                                drive.withRotationalRate(
                                                                headingPid.calculate(
                                                                                swerve.getState().Pose.getRotation()
                                                                                                .getDegrees(),
                                                                                                target
                                                                                                .getDegrees()))),
                                swerve).until(() -> headingPid.atSetpoint()).andThen(() -> headingPid.close());

        }


        public static Rotation2d calculateAngleToPoint(Translation2d targetPose) {

                Pose2d currentPose = swerve.getState().Pose;

                // Get the difference in x and y positions
                double dx = targetPose.getX() - currentPose.getX();
                double dy = targetPose.getY() - currentPose.getY();

                // Use atan2 to calculate the angle between the two points
                double angleToTargetRadians = Math.atan2(dy, dx);

                // Return the angle as a Rotation2d
                return new Rotation2d(angleToTargetRadians + Units.degreesToRadians(180));
        }

                public static Rotation2d calculateAngleToPass() {

                Pose2d currentPose = swerve.getState().Pose;
                Translation2d targetPose = Constants.passPosePoint;

                // Get the difference in x and y positions
                double dx = targetPose.getX() - currentPose.getX();
                double dy = targetPose.getY() - currentPose.getY();

                // Use atan2 to calculate the angle between the two points
                double angleToTargetRadians = Math.atan2(dy, dx);

                // Return the angle as a Rotation2d
                return new Rotation2d(angleToTargetRadians + Units.degreesToRadians(180));
        }

}
