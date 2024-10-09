// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.CommandGroupFactory;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Vision.AprilTagCamera;
import frc.robot.subsystems.Vision.ObjectDetectionCamera;
import frc.robot.subsystems.Vision.Vision_Constants;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;  

  private final boolean UseLimelight = false;
  //left camera:
  private AprilTagCamera leftCamera1  = new AprilTagCamera(Vision_Constants.K_LEFT_CAMERA_NAME);
  //right camera:
  private AprilTagCamera rightCamera1  = new AprilTagCamera(Vision_Constants.K_RIGHT_CAMERA_NAME);
  //note camera:
  private ObjectDetectionCamera noteCamera1  = new ObjectDetectionCamera(Vision_Constants.K_NOTE_CAMERA_NAME);

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    
  }
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    System.out.println(leftCamera1.getAngleFromTarget());

    SmartDashboard.putNumber("angle from left camera:",leftCamera1.getAngleFromTarget()); 
    SmartDashboard.putNumber("angle from right camera:",rightCamera1.getAngleFromTarget()); 
    SmartDashboard.putNumber("angle from note camera:",noteCamera1.getAngleFromTarget()); 


    /**
     * This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to target
     * and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible, though exact implementation
     * of how to use vision should be tuned per-robot and to the team's specification.
     */
    }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {

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