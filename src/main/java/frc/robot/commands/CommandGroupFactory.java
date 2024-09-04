package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.Climb.ClimbSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.ShooterArmFolder.ShooterArmSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Commands;

public class CommandGroupFactory {
    private static final IntakeSubsystem intake = IntakeSubsystem.getInstance();
    private static final ClimbSubsystem climb = ClimbSubsystem.getInstance();
    private static final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    private static final ShooterArmSubsystem shooterArm = ShooterArmSubsystem.getInstance();

    public static Command shootFromBase(){
        return new ParallelDeadlineGroup(Commands.waitSeconds(1.5)// waits one rio scyle
        .andThen(Commands.waitUntil(() -> shooter.isAtVelocity() && shooterArm.isArmReady())
        .andThen(() -> intake.feedShooterCommand())), shooterArm.moveArmToBase(),shooter.shootFromBase());
    
        }
    
    public static Command yeet(){
        System.out.println("hello");
        return new ParallelDeadlineGroup(Commands.waitSeconds(2).andThen(() -> intake.feedShooterCommand()),
         shooter.setCurrentYeetCommand());

    }
    }



//Commands.waitUntil(() -> shooter.isAtVelocity())).andThen(() -> shooter.shootFromBase().andThen(() -> intake.feedShooterCommand())));


 
