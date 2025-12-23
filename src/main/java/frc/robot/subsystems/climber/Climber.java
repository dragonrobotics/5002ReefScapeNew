package frc.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberIO.ClimberInputs;

public class Climber extends SubsystemBase {
    private final ClimberIO climberIO;
    private final ClimberInputs climberInputs = new ClimberInputs();

    private Pose3d climberPose3d;

    private final StructPublisher<Pose3d> climberPosePublisher = NetworkTableInstance.getDefault().getStructTopic(
        "Climber Pose", 
        Pose3d.struct
    ).publish();

    public Climber(ClimberIO climberIO) {
        this.climberIO = climberIO;
    }

    public Command runClimber(double speed) {
        return run(
            () -> climberIO.runClimber(speed)
        );
    }

    public Command stopClimber() {
        return run(
            () -> climberIO.stopClimber()
        );
    }

    public Command climb() {
        return runClimber(6);
    }

    public Command release() {
        return runClimber(-6);
    }

    public Command resetEncoder() {
        return run(
            () -> climberIO.resetEncoder()
        );
    }

    @Override
    public void periodic() {
        climberIO.updateInputs(climberInputs);

        climberPose3d = new Pose3d(
            0, 
            -0.31115, 
            0.3651504, 
            new Rotation3d(
                climberInputs.angle, 
                0, 
                0
            )    
        );

        climberPosePublisher.set(climberPose3d);

        SmartDashboard.putNumber("Climber Angle", climberInputs.angle);
        SmartDashboard.putNumber("Climber Velocity", climberInputs.velocity);

        SmartDashboard.putNumber("Climber Amps", climberInputs.motorCurrent);
    }
}
