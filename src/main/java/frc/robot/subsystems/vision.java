package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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

        // transform from robot center to camera (forward, left, up)
        robotToCam = new Transform3d(
            new Translation3d(0.33, 0.10, 0.30),
            new Rotation3d(0.0, 0.0, 0.0)
        );

        // AprilTag field layout
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // PhotonPoseEstimator (use multi-tag PnP on coprocessor)
        // NOTE: This constructor variant (without camera param) matches older Photon API. If your Photon
        // library requires a constructor including the PhotonCamera, swap to that overload.
        poseEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCam
        );
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

        // === Simulation setup ===
        try {
            cameraProps = new SimCameraProperties();
            // Example calibration; tweak resolution/rotation to match your real camera if needed
            cameraProps.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraSim = new PhotonCameraSim(camera, cameraProps);

            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(fieldLayout);
            visionSim.addCamera(cameraSim, robotToCam);
        } catch (Exception e) {
            System.err.println("[vision] Simulation setup failed: " + e.getMessage());
            visionSim = null;
            cameraSim = null;
            cameraProps = null;
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

    public Optional<Pose2d> getBestTagFieldPose() {
        return getBestTarget()
            .flatMap(target -> fieldLayout.getTagPose(target.getFiducialId()))
            .map(Pose3d::toPose2d);
    }

    @Override
    public void periodic() {
        // update lastTarget for dashboard / debugging
        getBestTarget().ifPresentOrElse(t -> lastTarget = t, () -> lastTarget = null);

        // Display target presence
        SmartDashboard.putBoolean("Vision/HasTarget", lastTarget != null);
        SmartDashboard.putString("Vision/TargetID",
            lastTarget != null ? String.valueOf(lastTarget.getFiducialId()) : "None"
        );

        // Try to compute a global pose and feed it to drivetrain
        try {
            // Use the drivetrain's current pose (this is the reference used by your auto builder and CTRE internals)
            Pose2d referencePose = drivetrain.getState().Pose;

            // Get the EstimatedRobotPose from Photon (if any)
            Optional<EstimatedRobotPose> estimateOpt = getEstimatedGlobalPose(referencePose);

            if (estimateOpt.isPresent()) {
                EstimatedRobotPose estimate = estimateOpt.get();
                Pose2d estimated2d = estimate.estimatedPose.toPose2d();

                // Compute FPGA timestamp for when the frame was captured by using pipeline latency.
                // PhotonPipelineResult provides latency in ms — we retrieve the latest result to get latency.
                // Fallback: use current FPGA time if latency not available.
                double fpgaTimestamp = Timer.getFPGATimestamp(); // default fallback

                try {
                    Optional<PhotonPipelineResult> resOpt = getResult();
                    if (resOpt.isPresent()) {
                        PhotonPipelineResult res = resOpt.get();
                        // latency in milliseconds -> convert to seconds
                        // double latencySeconds = res.getLatencyMillis() / 1000.0;
                        fpgaTimestamp = Timer.getFPGATimestamp();
                        //  - latencySeconds;
                    }
                } catch (Exception ex) {
                    // if anything fails here, fallback to current FPGA time
                    fpgaTimestamp = Timer.getFPGATimestamp();
                }

                // Feed the vision measurement into the drivetrain odometry/kalman
                try {
                    drivetrain.addVisionMeasurement(estimated2d, fpgaTimestamp);
                } catch (Exception e) {
                    System.err.println("[vision] drivetrain.addVisionMeasurement failed: " + e.getMessage());
                }
            }
        } catch (Exception e) {
            System.err.println("[vision] periodic failed: " + e.getMessage());
        }
    }

    @Override
    public void simulationPeriodic() {
        if (visionSim != null && cameraSim != null && drivetrain != null) {
            try {
                // Use the drivetrain's simulated pose (field-relative)
                Pose2d simPose = drivetrain.getState().Pose;

                // Update vision sim with the robot pose BEFORE producing the camera frame
                visionSim.update(simPose);

                // Update the camera sim frame (PhotonCameraSim API provides update())
                // cameraSim.update();
            } catch (Exception e) {
                System.err.println("[vision] Simulation update failed: " + e.getMessage());
            }
        }
    }
}
