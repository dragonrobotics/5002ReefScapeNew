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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class vision extends SubsystemBase{
    private Pose2d prevPose2d;
        private final PhotonCamera camera; 
        private final Paths pather;
        private final AprilTagFieldLayout aprilTagFieldLayout;
        private final Transform3d robotToCam;
        private final PhotonPoseEstimator photonPoseEstimator;
        private final VisionSystemSim visionSim;
        private final CommandSwerveDrivetrain drivetrain;
        private final SimCameraProperties cameraProp;
        private final PhotonCameraSim cameraSim;
        private final CommandXboxController controller;
        private final TargetModel targetModel;
        private final PIDController xController;
        private final PIDController yController;
        private final ProfiledPIDController thetaController;
        private final HolonomicDriveController holonomicController;
        private PhotonTrackedTarget cachedResult;
        
                public vision(CommandSwerveDrivetrain drivetrain, CommandXboxController controller){ 
                    
                    prevPose2d = drivetrain.getState().Pose;
                    this.controller = controller;
                    
        
                    camera = new PhotonCamera(Constants.OperatorConstants.cameraName);
                    aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        
                    xController  = new PIDController(2.0, 0.0, 0.0);
                    yController = new PIDController(0, 0, 0);
                    thetaController = new ProfiledPIDController(3.0, 0.0, 0.0, new TrapezoidProfile.Constraints(2.0, 3.0));
                    holonomicController = new HolonomicDriveController(xController, yController, thetaController);
            
                    robotToCam = new Transform3d(new Translation3d(0.3302, 0.1016, 0.3048), new Rotation3d(0,0,0));
                    photonPoseEstimator =  new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
                    photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
                    pather = new Paths(drivetrain);
                    visionSim = new VisionSystemSim("main");
                    visionSim.addAprilTags(aprilTagFieldLayout);
                    targetModel = new TargetModel(0.1651, 0.1651);
                    visionSim.addAprilTags(Constants.OperatorConstants.layout);
                    cameraProp = new SimCameraProperties();
                    cameraSim = new PhotonCameraSim(camera, cameraProp);
                    cachedResult = null;
        
                    visionSim.addCamera(cameraSim, robotToCam);
            
                    this.drivetrain = drivetrain;
            
                }
            
                public PhotonPipelineResult getResult(){
                    return camera.getLatestResult();
                }
        
        
            
                public PhotonTrackedTarget getTracked(){
                    PhotonPipelineResult result = getResult();
                    if (result != null){
                        return result.getBestTarget();
                    }
                    else{
                        return null;
                    }
                }
            
                public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
                    PhotonPipelineResult result = getResult();
            
                    if (prevEstimatedRobotPose != null){
                        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose); 
                    }
                    if (result != null){
                        return photonPoseEstimator.update(result);
                    }
                    else{
                        return Optional.empty();
                    }
                }
        
                public Pose2d fixResult(){ // Buttons stop working after first command
                    PhotonTrackedTarget result = getTracked();
                    Transform3d pose;
                    if (result ==null){
                            result = cachedResult;
                    }
                    pose = result.getBestCameraToTarget();
                    double x = pose.getX();
                    double y = pose.getY();

            Pose2d pose2D = drivetrain.getState().Pose;
        
            Pose2d newPose2D = Paths.transformOffset(pose2D, y * 39.3701, -x * 39.3701, 0);
            return newPose2D;


        }
    
    @Override
    public void periodic() {
        PhotonPipelineResult result = getResult();
        if (result != null && result.hasTargets()) {
            photonPoseEstimator.setReferencePose(drivetrain.getState().Pose);
            Optional<EstimatedRobotPose> estimation = photonPoseEstimator.update(result);

        if (estimation.isPresent()) {
            Pose2d pose = estimation.get().estimatedPose.toPose2d();
            double timestamp = result.getTimestampSeconds();

            drivetrain.addVisionMeasurement(pose, timestamp);

            // Logger.recordOutput("Vision/EstimatedPose", pose);
            // Logger.recordOutput("Vision/HasTargets", result.hasTargets());
            // Logger.recordOutput("Vision/TagCount", result.getTargets().size());
            // Logger.recordOutput("Vision/Timestamp", timestamp);
        }

    }

    SmartDashboard.putNumber("Joystick X", controller.getLeftX());
    SmartDashboard.putNumber("Joystick Y", controller.getLeftY());
    
}

@Override
public void simulationPeriodic() {
    
    visionSim.update(drivetrain.getState().Pose);
}
}
