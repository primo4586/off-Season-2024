package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

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
import frc.robot.Constants;
import frc.robot.subsystems.Climb.ClimbSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.ShooterArmFolder.ShooterArmSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.TunerConstants;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class CommandGroupFactory {
        private static final IntakeSubsystem intake = IntakeSubsystem.getInstance();
        private static final ClimbSubsystem climb = ClimbSubsystem.getInstance();
        private static final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
        private static final ShooterArmSubsystem shooterArm = ShooterArmSubsystem.getInstance();
        private static final CommandSwerveDrivetrain swerve = TunerConstants.DriveTrain;

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
                return new ParallelCommandGroup(shooter.shootFromFar());
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
                return new ParallelDeadlineGroup(Commands.waitSeconds(2).andThen(intake.feedShooterCommand()),
                                shooter.setShooterSpeed(25));
        }

        public static Command shotSpeakerCommand() {
                return new ParallelDeadlineGroup(
                                alignSpeakerCommand()
                                                .andThen(Commands.waitUntil(
                                                                () -> shooter.isAtVelocity() && shooterArm.isArmReady())
                                                                .andThen(intake.feedShooterCommand())),

                                shooterArm.speakerAngleExterapolateCommand(Constants.distanceFromSpeaker),

                                shooter.shootFromFar());
        }

        public static Command alignSpeakerCommand() {

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
                                                                                calculateAngleToSpeaker()
                                                                                                .getDegrees()))),
                                swerve).until(() -> headingPid.atSetpoint()).andThen(() -> headingPid.close());

        }

        public static Rotation2d calculateAngleToSpeaker() {

                Pose2d currentPose = swerve.getState().Pose;
                Translation2d targetPose = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                                ? Constants.speakerPoseBlue
                                : Constants.speakerPoseRed;

                // Get the difference in x and y positions
                double dx = targetPose.getX() - currentPose.getX();
                double dy = targetPose.getY() - currentPose.getY();

                // Use atan2 to calculate the angle between the two points
                double angleToTargetRadians = Math.atan2(dy, dx);

                // Return the angle as a Rotation2d
                return new Rotation2d(angleToTargetRadians + Units.degreesToRadians(180));
        }

        // TODO: add all swerve vision and interpolation commands
}
