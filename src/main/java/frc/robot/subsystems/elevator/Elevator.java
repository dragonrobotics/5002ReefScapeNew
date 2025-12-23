package frc.robot.subsystems.elevator;

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
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorInputs;

public class Elevator extends SubsystemBase {
    private final ElevatorIO elevatorIO;
    private final ElevatorInputs elevatorInputs = new ElevatorInputs();

    private final ProfiledPIDController elevatorPID = new ProfiledPIDController(
        PID.Elevator.kP,
        PID.Elevator.kI,
        PID.Elevator.KD,
        new TrapezoidProfile.Constraints(
            PID.Elevator.maxVelocity,
            PID.Elevator.maxAcceleration
        )
    );

    // Publishes the poses declared below.
    StructPublisher<Pose3d> firstStagePosePublisher = NetworkTableInstance.getDefault().getStructTopic(
        "Elevator First Stage Pose", 
        Pose3d.struct
    ).publish();

    StructPublisher<Pose3d> secondStagePosePublisher = NetworkTableInstance.getDefault().getStructTopic(
        "Elevator Second Stage Pose", 
        Pose3d.struct
    ).publish();

    // Visualizes the elevator during simulation.
    private Pose3d firstStagePose3d, secondStagePose3d;

    public ElevatorSetpoint elevatorSetpoint;
    
    public enum ElevatorSetpoint {
        Default(1),
        Intake(19.87),
        Climb(0.5),

        L2(7.31),
        L3(15.25),
        L4(27.95),

        A2(4.5),
        A3(12.25);

        public double height;

        private ElevatorSetpoint(double height) {
            this.height = height;
        }
    }

    public Elevator(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;

        elevatorPID.setTolerance(PID.Elevator.tolerance);
    }

    public Command runElevator(double speed) {
        return run(
            () -> elevatorIO.runElevator(speed)
        );
    }

    public Command stopElevator() {
        return runOnce(
            () -> elevatorIO.stopElevator()
        );
    }

    public Command moveElevatorUp() {
        return runElevator(0.3);
    }

    public Command moveElevatorDown() {
        return runElevator(-0.3);
    }

    public Command calibrate() {
        return runOnce(
            () -> elevatorIO.resetEncoder()
        );
    }

    public Command goToHeight(ElevatorSetpoint elevatorSetpoint) {
        this.elevatorSetpoint = elevatorSetpoint;

        elevatorPID.reset(
            elevatorInputs.height, 
            elevatorInputs.velocity
        );

        elevatorPID.setGoal(elevatorSetpoint.height);

        return runElevator(
            elevatorPID.calculate(elevatorInputs.height)
        );
    }

    @Override
    public void periodic() {
        elevatorIO.updateInputs(elevatorInputs);

        firstStagePose3d = new Pose3d(
            0, 
            -0.13335, 
            Units.inchesToMeters(elevatorInputs.height) + 0.1873504,
            new Rotation3d(
                0, 
                0, 
                0
            )
        );

        secondStagePose3d = new Pose3d(
            0,
            -0.13335, 
            Units.inchesToMeters(elevatorInputs.height * 2) + 0.1873504, 
            new Rotation3d(
                0, 
                0, 
                0
            )
        );

        firstStagePosePublisher.set(firstStagePose3d);
        secondStagePosePublisher.set(secondStagePose3d);

        SmartDashboard.putData("Elevator PID Controller", elevatorPID);

        SmartDashboard.putNumber("Elevator Height", elevatorInputs.height);
        SmartDashboard.putNumber("Elevator Velocity", elevatorInputs.velocity);

        SmartDashboard.putNumber("Elevator Motor Amps", elevatorInputs.motorCurrent);
    }
}