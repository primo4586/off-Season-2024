// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.SmartDashbored.NetworkTabels;
import frc.robot.commands.CommandGroupFactory;
import frc.robot.subsystems.Vision.AprilTagCamera;
import frc.robot.subsystems.Vision.Vision_Constants;
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.util.shuffleboardAlike.AutoContainer;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  
  private RobotContainer m_robotContainer;
  private AutoContainer autoContainer;

  private AprilTagCamera vision;
  

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    autoContainer = new AutoContainer();
    NetworkTabels.setClock(150, 0); 

    

    // TunerConstants.DriveTrain.getDaqThread().setThreadPriority(99);
    //     SignalLogger.setPath("/media/sda1/");
        // SignalLogger.start();
  }
  @Override //0.02
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    NetworkTabels.updateValues();
    }

  @Override
  public void disabledInit() {
    NetworkTabels.setClock(150, 0);
  }

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

    NetworkTabels.setClock(15, 1);
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
    NetworkTabels.setClock(135, 1);
  }

  @Override
  public void teleopPeriodic() {
  }

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