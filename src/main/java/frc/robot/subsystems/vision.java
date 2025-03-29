package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class vision extends SubsystemBase{
    private final PhotonCamera camera; 
    private final AprilTagFieldLayout aprilTagFieldLayout;
    private final Transform3d robotToCam;
    private final PhotonPoseEstimator photonPoseEstimator;

    private final CommandSwerveDrivetrain drivetrain;

    public vision(CommandSwerveDrivetrain drivetrain){ 
       camera = new PhotonCamera(Constants.OperatorConstants.cameraName);
       aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
       robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
       photonPoseEstimator =  new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);

       this.drivetrain = drivetrain;

    }

    public PhotonPipelineResult getResult(){
        return camera.getLatestResult();
    }

    public PhotonTrackedTarget getTracked(){
        if (getResult() != null){
            return getResult().getBestTarget();
        }
        else{
            return null;
        }
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose); 
        return photonPoseEstimator.update(getResult());
    }

    @Override
    public void periodic(){
        if (getResult().hasTargets()){
            Optional<EstimatedRobotPose> estimation = getEstimatedGlobalPose(drivetrain.getState().Pose);
            Pose2d pose = estimation.get().estimatedPose.toPose2d();
            var timestamp = estimation.get().timestampSeconds;
            System.out.println(timestamp);
            drivetrain.addVisionMeasurement(pose, timestamp);
        }
    }
}
