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
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AprilTagPoses;
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


    /**
     * Defines the different modes that the robot can use to align itself with an april tag.
     * 
     * <p> {@link #FOLLOW}: Aligns the robot to an april tag based on a continuous flow of data from its camera. Use this while testing the robot outside of competitions.
     * <p> {@link #PATHFIND}: Aligns the robot to an april tag where position/rotation data is predefined by the field layout. Use this during competition for reliability.
     */
    public enum AprilTagAlignMode {
        FOLLOW,
        PATHFIND
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

    /**
     * Converts the raw april tag output of the robot's camera into a usable set of position and rotation attributes.
     * 
     * @param aprilTagAlignMode
     * The way in which you want the robot to align to a specific april tag. Refer to {@link AprilTagAlignMode} for more info.
     * @return
     * A Pose2d object that the robot can drive to via. a Holonomic Drive Controller or through PathPlanner's solutions.
     */
    public Optional<Pose2d> getTrackedAprilTagPose(AprilTagAlignMode aprilTagAlignMode) {
        return getTrackedAprilTag().flatMap(
            trackedAprilTag -> {
                switch (aprilTagAlignMode) {
                    case FOLLOW -> {
                        final Transform3d cameraToTag = trackedAprilTag.getBestCameraToTarget();

                        final Transform2d robotToCamera = new Transform2d(
                            cameraPoseOffset.getTranslation().toTranslation2d(),
                            cameraPoseOffset.getRotation().toRotation2d()
                        );

                        final Transform2d cameraToTag2d = new Transform2d(
                            cameraToTag.getTranslation().toTranslation2d(),
                            cameraToTag.getRotation().toRotation2d()
                        );

                        final Transform2d tagToGoal = new Transform2d(
                            new Translation2d(
                                Units.inchesToMeters(20),
                                0
                            ),
                            Rotation2d.fromDegrees(180)
                        );

                        return Optional.of(
                            swerveDrive.getState().Pose
                                .transformBy(robotToCamera)
                                .transformBy(cameraToTag2d)
                                .transformBy(tagToGoal)
                        );
                    }
                    
                    case PATHFIND -> {
                        final int trackedAprilTagID = trackedAprilTag.getFiducialId();

                        final Pose2d trackedAprilTagPose = AprilTagPoses.tagLayout.getTagPose(trackedAprilTagID).get().toPose2d();

                        final Translation2d loggedPoseTranslation = trackedAprilTagPose.getTranslation().plus(
                            new Translation2d(
                                Units.inchesToMeters(20),
                                0
                            ).rotateBy(
                                trackedAprilTagPose.getRotation()
                            )
                        );

                        final Rotation2d loggedPoseRotation = trackedAprilTagPose.getRotation().plus(
                            AprilTagPoses.intakePoseTagIDs.contains(trackedAprilTagID) ?
                                Rotation2d.fromDegrees(0) :
                                Rotation2d.fromDegrees(180)   
                        );

                        return Optional.of(
                            new Pose2d(
                                loggedPoseTranslation,
                                loggedPoseRotation
                            )
                        );
                    }
                }

                return Optional.empty();
            }
        );
    }

    public Command driveToTrackedAprilTag(AprilTagAlignMode aprilTagAlignMode) {        
        return Commands.defer(
            () -> {
                Optional<Pose2d> tagPoseOptional = getTrackedAprilTagPose(aprilTagAlignMode);

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

        getTrackedAprilTagPose(AprilTagAlignMode.PATHFIND).ifPresent(
            pose -> DogLog.log("Target Vision Pathfinding Pose", pose)
        );

        getTrackedAprilTagPose(AprilTagAlignMode.FOLLOW).ifPresent(
            pose -> DogLog.log("Target Vision Follow Pose", pose)
        );
    }

    @Override
    public void simulationPeriodic() {
        visionSystemSim.update(swerveDrive.getState().Pose);
    }
}
