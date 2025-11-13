package frc.robot.subsystems;

import java.security.DrbgParameters.Reseed;
import java.util.Optional;

//import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.jni.SwerveJNI.DriveState;

import org.photonvision.PhotonUtils;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.ExponentialProfile.State;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;

public class vision extends SubsystemBase {
    private final PhotonCamera camera;
    private final Transform3d robotToCam;
    private final AprilTagFieldLayout fieldLayout;
    private final PhotonPoseEstimator poseEstimator;
    private final CommandSwerveDrivetrain drivetrain; // Needed for sim pose updates

    // Simulation
    private VisionSystemSim visionSim;
    private PhotonCameraSim cameraSim;
    private SimCameraProperties cameraProps;

    private PhotonTrackedTarget lastTarget;

    public vision(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        // === Real camera setup ===
        camera = new PhotonCamera(Constants.OperatorConstants.cameraName);
        robotToCam = new Transform3d(
            new Translation3d(0.33, 0.10, 0.30),
            new Rotation3d(0, 0, 0)
        );

        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        poseEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCam
        );
        poseEstimator.setMultiTagFallbackStrategy(
            PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE
        );

        // === Simulation setup ===
        try {
            cameraProps = new SimCameraProperties();
            cameraProps.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraSim = new PhotonCameraSim(camera, cameraProps);
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(fieldLayout);
            visionSim.addCamera(cameraSim, robotToCam);
        } catch (Exception e) {
            System.err.println("[vision] Simulation setup failed: " + e.getMessage());
        }

        lastTarget = null;
    }

    /** Get latest PhotonVision result safely */
    public Optional<PhotonPipelineResult> getResult() {
        try {
            PhotonPipelineResult result = camera.getLatestResult();
            return (result != null) ? Optional.of(result) : Optional.empty();
        } catch (Exception e) {
            System.err.println("[vision] Error getting result: " + e.getMessage());
            return Optional.empty();
        }
    }

    /** Return best tracked target if available */
    public Optional<PhotonTrackedTarget> getBestTarget() {
        return getResult()
            .filter(PhotonPipelineResult::hasTargets)
            .map(PhotonPipelineResult::getBestTarget);
    }

    
    /** Get AprilTag field pose for current target */
    public Optional<Pose2d> getTrackedTagPose() {
        return getBestTarget().flatMap(target -> {
            try {
                int id = target.getFiducialId();
                Optional<Pose3d> tagPose = fieldLayout.getTagPose(id);
                return tagPose.map(Pose3d::toPose2d);
            } catch (Exception e) {
                System.err.println("[vision] Tag pose lookup failed: " + e.getMessage());
                return Optional.empty();
            }
        });
    }

    /** Estimate global pose given a previous estimate */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevPose) {
        return getResult().flatMap(result -> {
            try {
                if (prevPose != null) poseEstimator.setReferencePose(prevPose);
                return poseEstimator.update(result);
            } catch (Exception e) {
                System.err.println("[vision] Pose estimation failed: " + e.getMessage());
                return Optional.empty();
            }
        });
    }

    @Override
    public void periodic() {
        getBestTarget().ifPresentOrElse(
            t -> lastTarget = t,
            () -> lastTarget = null
        );

        SmartDashboard.putBoolean("Vision/HasTarget", lastTarget != null);
        SmartDashboard.putString("Vision/TargetID",
            lastTarget != null ? String.valueOf(lastTarget.getFiducialId()) : "None"
        );
    }

    @Override
    public void simulationPeriodic() {
        if (visionSim != null && drivetrain != null) {
            try {
                Pose2d simPose = drivetrain.getState().Pose;
                visionSim.update(simPose);
            } catch (Exception e) {
                System.err.println("[vision] Simulation update failed: " + e.getMessage());
            }
        }
    }
}