package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision;

public class AutoAlignToTagCommand extends Command {

    private enum AlignState {
        ACQUIRE,
        DRIVE
    }

    private final vision vision;
    private final CommandSwerveDrivetrain drivetrain;

    private AlignState state = AlignState.ACQUIRE;

    // Controllers (FIELD-RELATIVE)
    private final PIDController xController = new PIDController(2.2, 0.0, 0.35);
    private final PIDController yController = new PIDController(2.2, 0.0, 0.35);

    private final ProfiledPIDController thetaController =
        new ProfiledPIDController(
            5.0, 0.0, 0.3,
            new TrapezoidProfile.Constraints(
                Math.PI,
                2 * Math.PI
            )
        );

    private Pose2d goalPose;
    private final Timer acquireTimer = new Timer();

    private static final double ACQUIRE_TIMEOUT = 0.4;

    public AutoAlignToTagCommand(vision vision, CommandSwerveDrivetrain drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        xController.setTolerance(0.03);
        yController.setTolerance(0.03);
        thetaController.setTolerance(Math.toRadians(2.0));
    }

    @Override
    public void initialize() {
        state = AlignState.ACQUIRE;

        goalPose = null;          
        acquireTimer.reset();
        acquireTimer.start();
    
        xController.reset();
        yController.reset();
        thetaController.reset(
    drivetrain.getState().Pose.getRotation().getRadians()
);
    }

    @Override
    public void execute() {
        switch (state) {
            case ACQUIRE -> acquireTarget();
            case DRIVE   -> driveToGoal();
        }
    }

    private void acquireTarget() {
        // Freeze motion to help vision
        if (goalPose != null) {
            return; // already locked THIS press
        }
    
        drivetrain.setControl(
            drivetrain.getAutoAlignRequest()
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
    
        vision.getBestTagFieldPose().ifPresent(tagPose -> {
            goalPose = computeGoalPose(tagPose);
            state = AlignState.DRIVE;
        });
    }

    private void driveToGoal() {
        Pose2d currentPose = drivetrain.getState().Pose;

        double vx = xController.calculate(
            currentPose.getX(), goalPose.getX()
        );

        double vy = yController.calculate(
            currentPose.getY(), goalPose.getY()
        );

        double omega = thetaController.calculate(
            currentPose.getRotation().getRadians(),
            goalPose.getRotation().getRadians()
        );

        drivetrain.setControl(
            drivetrain.getAutoAlignRequest()
                .withVelocityX(-vx)
                .withVelocityY(-vy)
                .withRotationalRate(omega)
        );
    }

    private Pose2d computeGoalPose(Pose2d tagPose) {
        Transform2d tagToRobot = new Transform2d(
            new Translation2d(0.5, 0),
            Rotation2d.fromDegrees(180)
        );

        return tagPose.transformBy(tagToRobot);
    }

    @Override
    public boolean isFinished() {
        if (state == AlignState.ACQUIRE) {
            return acquireTimer.hasElapsed(ACQUIRE_TIMEOUT);
        }

        return xController.atSetpoint()
            && yController.atSetpoint()
            && thetaController.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(
            drivetrain.getAutoAlignRequest()
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
    }
}
