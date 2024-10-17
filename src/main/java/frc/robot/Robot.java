// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.CommandGroupFactory;
import frc.robot.subsystems.Vision.AprilTagCamera;
import frc.robot.subsystems.Vision.ObjectDetectionCamera;
import frc.robot.subsystems.Vision.Vision_Constants;
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.util.shuffleboardAlike.AutoContainer;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private AutoContainer autoContainer;
  private static AprilTagCamera leftAprilTagCamera = new AprilTagCamera(Vision_Constants.K_LEFT_CAMERA_NAME);
  private static ObjectDetectionCamera noteCamera = new ObjectDetectionCamera(Vision_Constants.K_NOTE_CAMERA_NAME);

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    autoContainer = new AutoContainer();

    TunerConstants.DriveTrain.getDaqThread().setThreadPriority(99);
        SignalLogger.setPath("/media/sda1/");
        SignalLogger.start();
  }
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("angle to speaker", CommandGroupFactory.calculateAngleToPoint(Constants.speakerPosePoint).getDegrees());
    SmartDashboard.putNumber("distance from spiker", Constants.distanceFromSpeaker.getAsDouble());
    SmartDashboard.putBoolean("see target",leftAprilTagCamera.seeTarget());
    SmartDashboard.putBoolean("see note",noteCamera.getDetectingObject());
    }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {

    m_autonomousCommand = autoContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}