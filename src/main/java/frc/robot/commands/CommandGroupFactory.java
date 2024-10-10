package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.ShooterArmFolder.ShooterArmSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.TunerConstants;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.swerve.TunerConstants;

public class CommandGroupFactory {
    private static final CommandXboxController driverJoystick = RobotContainer.driverJoystick;
    private static final IntakeSubsystem intake = IntakeSubsystem.getInstance();
    private static final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    private static final ShooterArmSubsystem shooterArm = ShooterArmSubsystem.getInstance();
    private static final CommandSwerveDrivetrain swerve = TunerConstants.DriveTrain ; // my drivetrain

    public static SwerveRequest.FieldCentricFacingAngle driveAlignedToSpeaker = new SwerveRequest.FieldCentricFacingAngle();
    static double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed


    public static Command shootFromBase(){
        return new ParallelDeadlineGroup(Commands.waitSeconds(0.02)// waits one rio scyle
        .andThen(Commands.waitUntil(() -> shooter.isAtVelocity() && shooterArm.isArmReady())
        .andThen(intake.feedShooterCommand())), shooterArm.moveArmToBase(),shooter.shootFromBase());
        }
    public static Command prepareToShoot(){// TODO: add shooter arm
        return new ParallelCommandGroup(shooter.shootFromFar());
    }

    //TODO: add swerve steer
    public static Command shootFromMidum(){
        return new ParallelDeadlineGroup(Commands.waitSeconds(0.02)// waits one rio scyle
        .andThen(Commands.waitUntil(() -> shooter.isAtVelocity() && shooterArm.isArmReady())
        .andThen(intake.feedShooterCommand())), shooterArm.moveArmToMedium(),shooter.shootFromBase());
        }
    
    public static Command yeet(){
        return new ParallelDeadlineGroup(Commands.waitSeconds(1.5).andThen(intake.feedShooterCommand()),
         shooter.setCurrentYeetCommand());
    }

    public static Command getDriveAlignedToSpeakerCommand() {//TODO: target direction
        return swerve
        .applyRequest(() -> driveAlignedToSpeaker
                        .withVelocityX(-driverJoystick.getLeftY() * MaxSpeed)
                        .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed)
                        .withTargetDirection(null))
                .ignoringDisable(true);
    }


    

    //TODO: add all swerve vision and interpolation commands
    }




//Commands.waitUntil(() -> shooter.isAtVelocity())).andThen(() -> shooter.shootFromBase().andThen(() -> intake.feedShooterCommand())));


 
