package frc.robot.subsystems;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import dev.doglog.DogLog;

import java.util.Optional;
import java.util.Set;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PhotonVision;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class Vision extends SubsystemBase {
    private final CommandSwerveDrivetrain swerveDrive;

    private final PhotonCamera camera = new PhotonCamera(PhotonVision.cameraName);

    private final Transform3d cameraPoseOffset = new Transform3d(
        new Translation3d(
            0.3302, 
            0.1016, 
            0.3048
        ),
        new Rotation3d(
            0,
            0,
            0
        )
    );

    // Photon stuff
    private final PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(
        Constants.AprilTagPoses.tagLayout, 
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        cameraPoseOffset
    );

    // Autoalign driving constraints
    private final PathConstraints tagAlignConstraints = new PathConstraints(
        2.4,
        4.0,
        Units.degreesToRadians(540),
        Units.degreesToRadians(720)
    );

    /*
    * Simulated photonvision components
    */
    private final VisionSystemSim visionSystemSim = new VisionSystemSim("main");
    private final SimCameraProperties simCameraProperties = new SimCameraProperties();
    private final PhotonCameraSim simCamera = new PhotonCameraSim(camera, simCameraProperties);

    public TagSideOffset tagSideOffset;

    public enum TagSideOffset {
        Left(-2),
        None(0),
        Right(2);

        public double yOffset;

        private TagSideOffset(double yOffset) {
            this.yOffset = yOffset;
        }
    }

    public Vision(CommandSwerveDrivetrain swerveDrive) {
        this.swerveDrive = swerveDrive;

        photonPoseEstimator.setMultiTagFallbackStrategy(
            PoseStrategy.CLOSEST_TO_REFERENCE_POSE
        );

        simCamera.enableRawStream(PhotonVision.simRawSteamEnabled);
        simCamera.enableProcessedStream(PhotonVision.simProcessedSteamEnabled);

        visionSystemSim.addAprilTags(Constants.AprilTagPoses.tagLayout);
        visionSystemSim.addCamera(simCamera, cameraPoseOffset);

        CameraServer.startAutomaticCapture();
    }

    public PhotonPipelineResult getCameraResult() {
        return camera.getLatestResult() ;
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        PhotonPipelineResult cameraResult = getCameraResult();

        if (prevEstimatedRobotPose != null) {
            photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        }

        if (cameraResult != null) {
            return photonPoseEstimator.update(cameraResult);
        } else {
            return Optional.empty();
        }
    }

    private Optional<PhotonTrackedTarget> getTrackedAprilTag() {
        PhotonPipelineResult cameraResult = getCameraResult();

        if (cameraResult != null && cameraResult.hasTargets()) {
            return Optional.of(cameraResult.getBestTarget());
        } else {
            return Optional.empty();
        }
    }

    private Optional<Pose2d> getTrackedAprilTagPose() {
        return getTrackedAprilTag().flatMap(
            target -> Constants.AprilTagPoses.tagLayout
                .getTagPose(target.getFiducialId())
                .map(pose3d -> {
                    Pose2d tagPose2d = pose3d.toPose2d();

                    Translation2d poseOffset = new Translation2d(
                        Units.inchesToMeters(18),
                        0
                    )
                    .rotateBy(
                        tagPose2d.getRotation()
                    );

                    Translation2d newPoseTranslation = tagPose2d.getTranslation().plus(poseOffset);

                    Rotation2d newPoseHeading = tagPose2d.getRotation().plus(
                        Rotation2d.fromDegrees(180)
                    );

                    return new Pose2d(
                        newPoseTranslation,
                        newPoseHeading
                    );
                }
            )
        );
    }


    public Command driveToTrackedAprilTag() {        
        return Commands.defer(
            () -> {
                Optional<Pose2d> tagPoseOptional = getTrackedAprilTagPose();

                if (tagPoseOptional.isEmpty()) {
                    return Commands.none();
                }
                        
                final Command tagAlignCommand = AutoBuilder.pathfindToPose(
                    tagPoseOptional.get(),
                    tagAlignConstraints,
                    0
                );

                return tagAlignCommand;
            }, 
            Set.of(this)
        );
    }

    @Override
    public void periodic() {
        PhotonPipelineResult cameraResult = getCameraResult();

        if (cameraResult != null && cameraResult.hasTargets()) {
            photonPoseEstimator.setReferencePose(swerveDrive.getState().Pose);
            Optional<EstimatedRobotPose> estimation = photonPoseEstimator.update(cameraResult);

            if (estimation.isPresent()) {
                Pose2d estimatedPose = estimation.get().estimatedPose.toPose2d();
                double cameraResultTimestamp = cameraResult.getTimestampSeconds();

                swerveDrive.addVisionMeasurement(estimatedPose, cameraResultTimestamp);
            }
        }

        getTrackedAprilTagPose().ifPresent(
            pose -> DogLog.log("Refined Tracked Pose", pose)
        );
    }

    @Override
    public void simulationPeriodic() {
        visionSystemSim.update(swerveDrive.getState().Pose);
    }
}
