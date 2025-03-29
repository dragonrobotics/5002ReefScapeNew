package frc.robot;

import java.math.RoundingMode;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.RobotContainer;

public class autoAlign extends Command {
    private final PhotonCamera camera;
    private final CommandSwerveDrivetrain swerveDrive;
    private final SwerveRequest.RobotCentric requester;

    private final PIDController rotationController;
    private final PIDController xPidController;
    private final PIDController yPidController;

    private final int targetID;
    
    private final Timer lostTargetTimer = new Timer();
        private CommandXboxController controller;
    
        private static final double LOST_TARGET_TIMEOUT = 0.5; 
    
        public autoAlign(CommandSwerveDrivetrain swerveDrive, CommandXboxController controller, int targetID) {
        this.controller = controller;
        this.swerveDrive = swerveDrive;
        this.targetID = targetID;
        addRequirements(swerveDrive);

        camera = new PhotonCamera(OperatorConstants.cameraName);
        requester = new SwerveRequest.RobotCentric();

        // PID Controllers with Tolerance
        rotationController = new PIDController(0.1, 0.01, 0.0);
        xPidController = new PIDController(0.1, 0.01, 0.0);
        yPidController = new PIDController(0.1, 0.01, 0.0);

        rotationController.setTolerance(1); 
        xPidController.setTolerance(0.02);     
        yPidController.setTolerance(0.02);    

    }

    @Override
    public void initialize() {
        //lostTargetTimer.reset();
        //lostTargetTimer.start();
    }

    @Override
    public void execute() {
        PhotonPipelineResult result = camera.getLatestResult();
        boolean hasTargets = result.hasTargets();

        if (hasTargets) {
            var target = result.getBestTarget();
            if (target.getFiducialId() == targetID) {
                Transform3d bestCameraToTarget = target.getBestCameraToTarget();
                double distance = OperatorConstants.distanceToTag;  

                double rError = target.getYaw();
                double xError = bestCameraToTarget.getX();
                double yError = bestCameraToTarget.getY();

                double OoverA = yError / xError;

                Math.atan(OoverA);

                double rOutput = rotationController.calculate(rError, 0);
                double yOutput = yPidController.calculate(yError, 0);
                double xOutput = xPidController.calculate(xError, 0);

                System.out.println("rError:" + rError + "||" + "yError:" + yError + "||" +"xError:" + xError);

                swerveDrive.setControl(requester.withRotationalRate(rOutput));
                lostTargetTimer.reset(); 
            }else{
                swerveDrive.setControl(requester.withRotationalRate(0));
            }
        }

        if (controller.rightBumper().getAsBoolean()) {
            end(true);
        }
    }
//hey
    @Override
    public boolean isFinished() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (result.hasTargets()) {
            var target = result.getBestTarget();
            if (target.getFiducialId() == targetID) {
                double rotation = target.getYaw();
                //Transform3d bestCameraToTarget = target.getBestCameraToTarget();
                //double xError = bestCameraToTarget.getX() - OperatorConstants.distanceToTag;
                //double yError = bestCameraToTarget.getY();
                //double rError = target.getYaw();

                return rotationController.atSetpoint() || !result.hasTargets();
            }
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.setControl(requester.withVelocityX(0)
                                        .withVelocityY(0)
                                        .withRotationalRate(0));
    }
}
