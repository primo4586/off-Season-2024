// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.swerve.TunerConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final String CAN_BUS_NAME="canBus";

  static Translation2d speakerPoseBlue = new Translation2d(0, 5.54);
    static Translation2d speakerPoseRed = new Translation2d(16.39, 5.54);

  public static DoubleSupplier distanceFromSpeaker = () -> TunerConstants.DriveTrain.getState().Pose.getTranslation().getDistance(
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? speakerPoseBlue
                    : speakerPoseRed);
}
