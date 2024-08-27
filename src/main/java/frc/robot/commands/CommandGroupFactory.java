package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.Climb.ClimbSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooterArm.ShooterArmSubsystem;
import edu.wpi.first.wpilibj2.command.Commands;

public class CommandGroupFactory {
    private IntakeSubsystem intake = IntakeSubsystem.getInstance();
    private ClimbSubsystem climb = ClimbSubsystem.getInstance();
    private ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    private ShooterArmSubsystem shooterArm = ShooterArmSubsystem.getInstance();

    public static Command ShootFromBase(){
        return new ParallelDeadlineGroup(Commands.waitSeconds(0.02)// one rio scyle
        .andThen(Commands.waitUntil(() -> shooter.isAtVelocity() && shooterArm.isArmReady))
        .andThen(() -> intake.feedShooterCommand), () -> shooter.shootFromBase())
        /*return new ParallelDeadlineGroup(
            Commands.waitSeconds(0.02).andThen(Commands.waitUntil(() -> (shooterArm.isArmReady()
            && (shooter.isAtVelocity())).andThen(intake.feedShooterCommand())), 
        shooter.shootFromBase(),shooterArm.moveArmToBase());
        */
        };
    }

//Commands.waitUntil(() -> shooter.isAtVelocity())).andThen(() -> shooter.shootFromBase().andThen(() -> intake.feedShooterCommand())));


 
