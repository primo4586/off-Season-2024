// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision.AprilTagCamera;
import frc.robot.subsystems.Vision.Vision_Constants;

public class SmartDashBoardSubsysytem extends SubsystemBase {
  public static double timer = 150;
  public static AprilTagCamera vision =  new AprilTagCamera(Vision_Constants.K_RIGHT_CAMERA_NAME);
  /** Creates a new SmartDashBoardSubsysytem. */
  public SmartDashBoardSubsysytem() {}

  @Override
  public void periodic() {
  timer -= 0.02;
  SmartDashboard.putNumber("timer",timer - (timer % 1));
  //SmartDashboard.putBoolean("Apriltag detection", vision.seeTarget());
  SmartDashboard.pose
  }

  public static void teleopTimer(){ timer = 135;}

  public static void autoTimer(){ timer = 15;}
}
