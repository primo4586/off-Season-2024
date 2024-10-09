package frc.robot.subsystems.swerve;
import frc.robot.subsystems.Vision.Vision_Constants;
import frc.robot.subsystems.Vision.AprilTagCamera;
import frc.robot.subsystems.Vision.ObjectDetectionCamera;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    private AprilTagCamera rightAprilTagCamera = new AprilTagCamera(Vision_Constants.K_RIGHT_CAMERA_NAME);
    private AprilTagCamera leftAprilTagCamera = new AprilTagCamera(Vision_Constants.K_LEFT_CAMERA_NAME);


    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void periodic() {
        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }

         // Correct pose estimate with vision measurements
        var rightVisionEst = rightAprilTagCamera.getEstimatedGlobalPose(); //latest estimated robot pose
        rightVisionEst.ifPresent(
                rightEst -> {
                    //2d estimated pose
                    var estPose = rightEst.estimatedPose.toPose2d();

                    SignalLogger.writeDoubleArray("right camera pose",
                            new double[] { estPose.getX(), estPose.getY(), estPose.getRotation().getDegrees() });

                    // Change our trust in the measurement based on the tags we can see
                    var estStdDevs = rightAprilTagCamera.getEstimationStdDevs(estPose);
                    this.addVisionMeasurement(rightEst.estimatedPose.toPose2d(), rightEst.timestampSeconds, estStdDevs);
                });

                //same to left:
        var leftVisionEst = leftAprilTagCamera.getEstimatedGlobalPose();
        leftVisionEst.ifPresent(
                leftEst -> {
                    var estPose = leftEst.estimatedPose.toPose2d();

                    SignalLogger.writeDoubleArray("left camera pose",
                            new double[] { estPose.getX(), estPose.getY(), estPose.getRotation().getDegrees() });

                    // Change our trust in the measurement based on the tags we can see
                    var estStdDevs = leftAprilTagCamera.getEstimationStdDevs(estPose);

                    this.addVisionMeasurement(leftEst.estimatedPose.toPose2d(), leftEst.timestampSeconds, estStdDevs);
                });
    }
}
