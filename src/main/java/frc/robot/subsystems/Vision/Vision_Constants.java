package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public interface Vision_Constants {
        String K_RIGHT_CAMERA_NAME = "right Camra";
        String K_LEFT_CAMERA_NAME = "Left Camera";
        String K_NOTE_CAMERA_NAME = "Note Camera";
        Transform3d K_RIGHT_ROBOT_TO_CAM = new Transform3d(new Translation3d(-0.16, -0.075, 0.45),
        new Rotation3d(- Units.degreesToRadians(-27.55), 0, Units.degreesToRadians(180)));

        Transform3d K_LEFT_ROBOT_TO_CAM = new Transform3d(new Translation3d(-0.171, 0.316, 0.40),
        new Rotation3d(Units.degreesToRadians(-20.2), 0, Units.degreesToRadians(180)));

// The layout of the AprilTags on the field
        AprilTagFieldLayout K_TAG_LAYOUT = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

// The standard deviations of our vision estimated poses, which affect
// correction rate
// (Fake values. Experiment and determine estimation noise on an actual robot.)
        Matrix<N3, N1> K_SINGLE_TAG_STD_DEVS = VecBuilder.fill(1, 1, 2);
        Matrix<N3, N1> K_MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);
        
        Pose2d target = new Pose2d(1, 1, new Rotation2d(Units.degreesToRadians(0)));

}