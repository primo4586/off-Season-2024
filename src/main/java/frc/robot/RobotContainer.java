// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.CommandGroupFactory;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.ShooterArmFolder.ShooterArmSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
    new CommandXboxController(0);
  private final CommandXboxController m_operatorController =
    new CommandXboxController(1);
  private final CommandXboxController m_testerController =
    new CommandXboxController(2);

  private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
  private  final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  private final ShooterArmSubsystem shooterArm = ShooterArmSubsystem.getInstance();
      

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //simple commands
    /* 
    System.out.println("hello world");
    m_testerController.b().onTrue(intake.feedShooterCommand());
    m_testerController.a().onTrue(CommandGroupFactory.yeet());
    m_testerController.y().onTrue(intake.coolectUntilNoteCommand());
    m_driverController.x().onTrue(CommandGroupFactory.shootFromBase());
    */
    m_testerController.rightBumper().whileTrue(shooterArm.sysIdDynamic(SysIdRoutine.Direction.kForward));

    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   * 
   */
  private void configureBindings() {
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
