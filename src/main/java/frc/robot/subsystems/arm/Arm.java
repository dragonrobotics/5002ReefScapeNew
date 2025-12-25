package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PID;
import frc.robot.subsystems.arm.ArmIO.ArmInputs;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorInputs;

public class Arm extends SubsystemBase {
    private final ArmIO armIO;
    private final ArmInputs armInputs = new ArmInputs();

    private final ElevatorIO elevatorIO;
    private final ElevatorInputs elevatorInputs = new ElevatorInputs();

    private final ProfiledPIDController armPID = new ProfiledPIDController(
        PID.Arm.kP,
        PID.Arm.kI,
        PID.Arm.kD,
        new TrapezoidProfile.Constraints(
            PID.Arm.maxVelocity,
            PID.Arm.maxAcceleration
        )
    );

    public ArmSetpoint armSetpoint;

    public enum ArmSetpoint {
        Default(0),
        Intake(147.43),
        Climb(-60),

        L2(-36),
        L3(-36),
        L4(-41.8),

        A2(-45),
        A3(-45);

        public double angle;

        private ArmSetpoint(double angle) {
            this.angle = angle;
        }
    }

    // Visualizes the pose in AdvantageScope
    private Pose3d armPose3d;

    private final StructPublisher<Pose3d> armPosePublisher = NetworkTableInstance.getDefault().getStructTopic(
        "Arm Pose",
        Pose3d.struct
    ).publish();

    public Arm(ArmIO armIO, ElevatorIO elevatorIO) {
        this.armIO = armIO;
        this.elevatorIO = elevatorIO;

        armPID.setTolerance(PID.Arm.tolerance);

        armPID.enableContinuousInput(0, 360);

        setRotationGoal(ArmSetpoint.Default);
    }

    public void runArm(double speed) {
        armIO.runArm(speed);
    }

    public void stopArm() {
        armIO.stopArm();
    }

    public boolean atGoal() {
        return armPID.atGoal();
    }

    public Command rotateClockwise() {
        return run(
            () -> runArm(1)
        );
    }

    public Command rotateCounterClockwise() {
        return run(
            () -> runArm(-1)
        );
    }

    public Command calibrate() {
        return runOnce(
            () -> armIO.resetEncoder()
        );
    }

    public Command setRotationGoal(ArmSetpoint armSetpoint) {
        return runOnce(
            () -> {
                this.armSetpoint = armSetpoint;

                armPID.reset(armInputs.angle, armInputs.velocity);
                armPID.setGoal(armSetpoint.angle);  
            }
        );
    }

    public Command rotateToSetpoint() {
        return run(
            () -> runArm(
                armPID.calculate(armInputs.angle)
            )
        );
    }

    @Override
    public void periodic() {
        elevatorIO.updateInputs(elevatorInputs);

        armIO.updateInputs(armInputs);

        SmartDashboard.putNumber("Arm Angle", armInputs.angle);
        SmartDashboard.putNumber("Arm Velocity", armInputs.velocity);

        SmartDashboard.putNumber("Arm Absolute Angle", armInputs.absAngle);

        SmartDashboard.putBoolean("Arm At Goal", atGoal());

        SmartDashboard.putNumber("Arm Motor Amps", armInputs.motorCurrent);

        SmartDashboard.putData("Arm PID Controller", armPID);


    }

    @Override
    public void simulationPeriodic() {
        armPose3d = new Pose3d(
            0, 
            -0.0047752, 
            Units.inchesToMeters(elevatorInputs.height * 2) + 0.1873504,
            new Rotation3d(
                0,
                Units.degreesToRadians(90 - 54.133228 + armInputs.angle), 
                0
            )
        );

        armPosePublisher.set(armPose3d);
    }
}
