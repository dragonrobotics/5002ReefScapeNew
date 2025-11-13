package frc.robot.subsystems;

import java.lang.Thread.State;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class autoAlign extends SubsystemBase{

    private final CommandSwerveDrivetrain driveTrain;
    private final vision visionSubsystem;

    private final PIDController xController;
    private final PIDController yController;
    private final ProfiledPIDController thetaController;

    private final HolonomicDriveController holonomicController;

    private static final double DISTANCE_TOLERANCE = 0.01;
    private static final double ANGLE_TOLERANCE = 0.01;
    


    public autoAlign(CommandSwerveDrivetrain driveTrain, vision visionSubsystem){
        this.driveTrain = driveTrain;
        this.visionSubsystem = visionSubsystem;
        

        xController = new PIDController(3.0, 0.0, 0.0);
        yController = new PIDController(3.0, 0.0, 0.0);
        thetaController = new ProfiledPIDController(5.0, 0.0, 0.0, 
                            new Constraints(3.0, 3.0));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        holonomicController = new HolonomicDriveController(xController, yController, thetaController);
    }

    // public align(Transform2d transform2d){
    //     driveTrain.applyRequest(()-> );
    // }
}

