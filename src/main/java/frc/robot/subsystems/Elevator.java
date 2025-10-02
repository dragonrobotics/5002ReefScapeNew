package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import java.lang.ModuleLayer.Controller;
import java.util.function.BooleanSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
//Hello
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.ExponentialProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

@Logged
public class Elevator extends SubsystemBase{

    //goes from 0 to 27.5

    final SparkMax m_elevator = new SparkMax(OperatorConstants.m_elevator, MotorType.kBrushless);
    final SparkMax m_follower = new SparkMax(OperatorConstants.m_elevatorFollower, MotorType.kBrushless);

    SparkMaxConfig mainConfig = new SparkMaxConfig();
    SparkMaxConfig followerConfig = new SparkMaxConfig();

    final RelativeEncoder encoder = m_follower.getEncoder();

    final SparkRelativeEncoderSim encoderSim = new SparkRelativeEncoderSim(m_elevator);
    
    PIDController controller = new PIDController(0.8, 0, 0.05);

    ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0.25, 0.1);

    Double factor = 0.0;
    Double goal = 0.0;

    /*
     * Simulates the elevator for AdvantageScope
     */
    
    private final DCMotor m_elevatorGearBox = DCMotor.getNEO(2);

    private final SparkMaxSim m_elevatorMotorSim = new SparkMaxSim(m_elevator, m_elevatorGearBox);

    private final ElevatorSim m_elevatorSim =
        new ElevatorSim(
            m_elevatorGearBox,
            3,
            Units.lbsToKilograms(15.222378),
            Units.inchesToMeters(0.75),
            Units.inchesToMeters(0), // Min. elevator height
            Units.inchesToMeters(28), // Max. elevator height
            true,
            Units.inchesToMeters(0), // Starts the elevator at the bottom
            0,
            0.0);

    public Elevator(){
        
        mainConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(80)
        .encoder
            .positionConversionFactor(OperatorConstants.elvatorConversionFactor)
            .velocityConversionFactor(OperatorConstants.elvatorConversionFactor/60);
            
        followerConfig
            .follow(m_elevator, true)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(80)
        .encoder
            .positionConversionFactor(OperatorConstants.elvatorConversionFactor)
            .velocityConversionFactor(OperatorConstants.elvatorConversionFactor/60);

        m_elevator.configure(mainConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        controller.reset();

        controller.setTolerance(0.25);

        moveToPosition(1.0);
    }

    //moves the elevator to a position in inches
    public void moveToPosition(Double position){
        controller.reset();
        goal = position;
    }

    public Command runElevator(){
        return run(()->{
            double desiredSpeed = controller.calculate(getMeasurement(), goal);
            m_elevator.setVoltage(desiredSpeed);});
    }

    public Command maintanElevator(){
        return run(()->{
            double desiredSpeed = feedforward.calculate(1, 1);
            m_elevator.setVoltage(desiredSpeed);});
    }

    public void setMotor(Double speed){
        m_elevator.set(speed);
    }

    public void stopMotor(){
        m_elevator.stopMotor();
    }

    public void zeroEncoder(){
        encoder.setPosition(0);
    }
    
    public double getMeasurement(){
        return encoder.getPosition();
    }

    public double getElevatorDistance(){
        return m_elevatorSim.getPositionMeters();
    }

    public double getAmps(){
        return m_elevator.getOutputCurrent();
    }

    public BooleanSupplier atTop(){
        return () -> getAmps() > 100.0;
    }

    public void calibrate(){
        factor = getMeasurement();
        factor = 28 / getMeasurement();
    }

    public BooleanSupplier atGoal(){
        return ()->(controller.atSetpoint());
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Elevator Position", encoder.getPosition());
        SmartDashboard.putNumber("Elevator Amps", getAmps());
        SmartDashboard.putNumber("Elevator Conversion Factor", factor);
        SmartDashboard.putNumber("Elevator Goal", goal);
    //  SmartDashboard.putData("Elevator FeedForward", (Sendable) feedforward);
        SmartDashboard.putData("Elevator PID", controller);
    }

    public void simulationPeriodic(){
        SmartDashboard.putNumber("Elevator Sim Height", Units.metersToInches(getElevatorDistance()));

        m_elevatorSim.setInput(m_elevatorMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

        m_elevatorSim.update(0.02);
        
        m_elevatorMotorSim.iterate(
            60 * (m_elevatorSim.getVelocityMetersPerSecond()/(Math.PI * 1.25)),
            RoboRioSim.getVInVoltage(),
            0.02
        );

        encoder.setPosition(
            Units.metersToInches(
                m_elevatorSim.getPositionMeters()
            )
        );

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(
                m_elevatorSim.getCurrentDrawAmps()
            )
        );

        DogLog.log("ZeroedComponentPoses", new Pose3d[] {new Pose3d()});

        DogLog.log("Elevator First Stage Pose 3D", new Pose3d[] {
            new Pose3d(
                0, 
                -0.13335, 
                getElevatorDistance() + 0.1873504,
                new Rotation3d(
                    0, 
                    0, 
                    0
                )
            )
        });

        DogLog.log("Elevator Carriage Pose 3D", new Pose3d[] {
            new Pose3d(
                0,
                -0.13335, 
                (getElevatorDistance() * 2) + 0.1873504, 
                new Rotation3d(
                    0, 
                    0, 
                    0
                )
            )
        });
    }
}
