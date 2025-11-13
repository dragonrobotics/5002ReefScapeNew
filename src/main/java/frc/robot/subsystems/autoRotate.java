package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import org.opencv.core.Size;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.vision;

public class autoRotate extends SubsystemBase{

        public Pose2d[] currentPose;
    
        private final CommandSwerveDrivetrain driveTrain;
        private final SwerveRequest.FieldCentric requester;
    
        private final vision vision;
    
        private final PIDController rPidController;
        private final PIDController xPidController;
        private final PIDController yPidController;

        private final Pose2d[] blueAutoPoses = Constants.blueAutoPoses;
        private final Pose2d[] redAutoPoses = Constants.redAutoPoses;
        
                // private CommandXboxController controller;
                private CommandXboxController controller;

                public Pose2d getClosestTag(){
                    Pose2d closestTag = new Pose2d();
                    Pose2d currentPose = driveTrain.getState().Pose;
                    Pose2d[] currentPoseColor;
                    double distance;
                    double finalDistance = 1000;
                    var color = DriverStation.getAlliance();
        
                    if (color.isPresent() && color.get() == DriverStation.Alliance.Blue){
                        currentPoseColor = Constants.autoAlignSide.get("Blue");
                    }
                    else if (color.isPresent() && color.get() == DriverStation.Alliance.Red){
                        currentPoseColor = Constants.autoAlignSide.get("Red");
                    
                    for (int i = 0; i < currentPoseColor.length; i++) {
                        distance = Math.sqrt(Math.pow(currentPoseColor[i].getX() - currentPose.getX(),2) + Math.pow(blueAutoPoses[i].getY() - currentPose.getY(),2));
        
                        if (distance < finalDistance){
                            finalDistance = distance;
                            closestTag = currentPoseColor[i];
                            }
                        }
                    }
        
                    return closestTag;
                }
        
                public Pose2d translateCoordinates(Pose2d originalPose, double degreesRotate, double distance){
                    double newX = originalPose.getX() + (Math.cos(Math.toRadians(degreesRotate))*distance);
                    double newY = originalPose.getY() + (Math.sin(Math.toRadians(degreesRotate))*distance);
        
                    return new Pose2d(newX, newY, originalPose.getRotation());
                }
        
                public Pose2d getAlignmentReefPose(boolean left){
                    Pose2d tagPose = getClosestTag();
                    Pose2d goalPose = translateCoordinates(tagPose, tagPose.getRotation().getDegrees(), 0.5);
                    goalPose = left ? translateCoordinates(goalPose, tagPose.getRotation().getDegrees() - 90, 0.1) : translateCoordinates(goalPose, tagPose.getRotation().getDegrees() + 90, 0.2);
        
                    return goalPose;
                }
            
                public autoRotate(CommandSwerveDrivetrain driveTrain, vision vision, CommandXboxController controller /*CommandXboxController controller*/){
            
                    this.vision = vision;
                    this.controller = controller;
    
            this.driveTrain = driveTrain;
            currentPose = new Pose2d[6];    
            requester = new SwerveRequest.FieldCentric();
    
            rPidController = new PIDController(0.15, 0.0, 0.0);
            xPidController = new PIDController(3, 0.0, 0.5);
            yPidController = new PIDController(3, 0.0, 0.5);
    
    
            xPidController.setTolerance(0.1);     
            yPidController.setTolerance(0.1); 
            rPidController.setTolerance(1);
        }
    
        public void moveToState(boolean left){

            Pose2d goalPose = getAlignmentReefPose(left);

            double rOutput = rPidController.calculate(driveTrain.getState().Pose.getRotation().getDegrees(), goalPose.getRotation().getDegrees());
            double xOutput = xPidController.calculate(driveTrain.getState().Pose.getX(), goalPose.getX());
            double yOutput = yPidController.calculate(driveTrain.getState().Pose.getX(), goalPose.getY());
    
            driveTrain.setControl(requester.withVelocityY(yOutput).withVelocityX(xOutput));
        
        }

        public Pose2d returnGoalPose2d(boolean left){
            return getAlignmentReefPose(left);
        }
    
        @Override
    
        public void periodic(){
            Pose2d closestPose = getClosestTag();
            if ((Math.abs(getAlignmentReefPose(false).getX() - driveTrain.getState().Pose.getX()) <= 0.5) &&
                (Math.abs(getAlignmentReefPose(false).getY() - driveTrain.getState().Pose.getY()) <= 0.5)){
                    controller.setRumble(RumbleType.kRightRumble, 1.0);
            }
            else if ((Math.abs(getAlignmentReefPose(true).getX() - driveTrain.getState().Pose.getX()) <= 0.5) &&
                    (Math.abs(getAlignmentReefPose(true).getY() - driveTrain.getState().Pose.getY()) <= 0.5)){
                    controller.setRumble(RumbleType.kLeftRumble, 1.0);
            }
            else{
                    controller.setRumble(RumbleType.kBothRumble, 0.0);
            }
        }

    }


