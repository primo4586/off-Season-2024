// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.CommandGroupFactory;
import frc.robot.subsystems.Climb.ClimbSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.ShooterArmFolder.ShooterArmSubsystem;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.TunerConstants;

public class RobotContainer {

  private IntakeSubsystem intake = IntakeSubsystem.getInstance();
  private ClimbSubsystem climb = ClimbSubsystem.getInstance();
  private ShooterArmSubsystem shooterArm = ShooterArmSubsystem.getInstance();
  private ShooterSubsystem shooter = ShooterSubsystem.getInstance();


  private double MaxSpeed =  1 * TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI;//1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driverController = new CommandXboxController(0); // My joystick
  private final CommandXboxController operaController = new CommandXboxController(1);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

    private DoubleSupplier slowMode = () -> driverController.rightBumper().getAsBoolean() ? 0.35 : 1;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    // swerve command
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * MaxSpeed * slowMode.getAsDouble()) // Drive forward with
            // negative Y (forward)
            .withVelocityY(-driverController.getLeftX() * MaxSpeed *  slowMode.getAsDouble()) // Drive left with negative X (left)
            .withRotationalRate(-driverController.getRightX() * MaxAngularRate *  slowMode.getAsDouble()) // Drive counterclockwise with negative
                                                                                // X (left)
        ));

    // driverController.leftBumper().whileTrue(drivetrain.applyRequest(() ->
    //   brake));
    //   driverController.b().whileTrue(drivetrain
    //   .applyRequest(() -> point.withModuleDirection(new
    //   Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));



    if (Utils.isSimulation()) {
    drivetrain.seedFieldRelative(new Pose2d(new Translation2d(),
    Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
    driverController.rightTrigger().whileTrue(drivetrain.applyRequest(()->new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity).withVelocityX(2.5)));
    
      // reset the field-centric heading on left bumper press
      driverController.y().onTrue(drivetrain.runOnce(() ->
      drivetrain.seedFieldRelative()));

    // driver command
    driverController.a().onTrue(CommandGroupFactory.collectUntilNote());
    driverController.start().onTrue(shooterArm.prepareHomeCommand());
    // driverController.back().onTrue(CommandGroupFactory.yeet());

    // driverController.rightTrigger().onTrue(CommandGroupFactory.shootFromBase());
    // driverController.leftTrigger().onTrue(CommandGroupFactory.yeet());
    // op command
    driverController.rightBumper().onTrue(CommandGroupFactory.shotSpeakerCommand());
    driverController.leftBumper().onTrue(CommandGroupFactory.shootFromBase());
    driverController.rightTrigger().onTrue(CommandGroupFactory.passNote());
    
    operaController.leftBumper().whileTrue(CommandGroupFactory.prepareToShoot());
    operaController.rightBumper().onTrue(CommandGroupFactory.yeet());
    operaController.povUp().whileTrue(intake.setCurrentCommand(IntakeConstants.PLAY_CURRENT)); 
    operaController.povDown().whileTrue(intake.setCurrentCommand(-IntakeConstants.PLAY_CURRENT)); 

    operaController.x().whileTrue(shooterArm.setSpeed(0.2));
    operaController.b().whileTrue(shooterArm.setSpeed(-0.2));

    climb.setDefaultCommand(climb.setSpeedCommand(
    () -> operaController.getRightY() *- 1,
    () -> operaController.getLeftY() * -1));

  }

  public RobotContainer() {
    configureBindings();
    // NamedCommands.registerCommand("intake",
    // CommandGroupFactory.collectUntilNote());
    // NamedCommands.registerCommand("shoot",
    // CommandGroupFactory.shotSpeakerCommand());

    NamedCommands.registerCommand("intake", Commands.waitSeconds(1));
    NamedCommands.registerCommand("shoot", Commands.waitSeconds(1));

		// driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
		// driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
		// driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
		// driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
  }
}
