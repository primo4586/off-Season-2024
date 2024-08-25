package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.Climb.ClimbSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Commands;

public class CommandGroupFactory {
    private IntakeSubsystem intake = IntakeSubsystem.getInstance();
    private ClimbSubsystem climb = ClimbSubsystem.getInstance();
    private ShooterSubsystem shooter = ShooterSubsystem.getInstance();

    public static Command ShootFromClose(){
        return ParallelDeadlineGroup(Commands.waitSeconds(0.02)
        .andThen(Commands.waitUntill(() -> shooter.isAtVelocity())
        .andThen(intake.feedShooterCommand())),
        new InstantCommand(() -> shooter.setSpeakerVel()); 
    }
//Commands.waitUntil(() -> shooter.isAtVelocity())).andThen(() -> shooter.shootFromBase().andThen(() -> intake.feedShooterCommand())));


}   
