package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import dev.doglog.DogLog;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.Constants.OperatorConstants;
@Logged
public class Arm extends SubsystemBase {
    
    final SparkMax m_armRotator = new SparkMax(OperatorConstants.m_armRotator, MotorType.kBrushless);

    SparkMaxConfig rotatorConfig = new SparkMaxConfig();

    final SparkRelativeEncoder encoder = (SparkRelativeEncoder) m_armRotator.getEncoder();
    final SparkAbsoluteEncoder rotatorAbsoluteEncoder = m_armRotator.getAbsoluteEncoder();

    ProfiledPIDController m_controller = new ProfiledPIDController(0.1, 0, 0.00, new TrapezoidProfile.Constraints(99, 99));
    public double goal = 0.0;
    double factor = 0.0;

    public SendableChooser<Boolean> brakeMode = new SendableChooser<Boolean>();

    private Elevator elevator = RobotContainer.elevator;

    /*
     * Simulates the arm for AdvantageScope
     */

    private final DCMotor m_armGearBoxSim = DCMotor.getNEO(1);

    private final SparkMaxSim m_armMotorSim = new SparkMaxSim(m_armRotator, m_armGearBoxSim);

    private final SingleJointedArmSim m_armSim = 
        new SingleJointedArmSim(
            m_armGearBoxSim,
            3, // Represents a 3:1 gear ratio
            SingleJointedArmSim.estimateMOI(
                Units.inchesToMeters(23.092), 
                Units.lbsToKilograms(2.5917853)
            ),
            Units.inchesToMeters(23.092), // arm length
            Units.degreesToRadians(-180), // The lowest angle our arm can aim towards.
            Units.degreesToRadians(180), // Limits the arm to vertical positioning.
            true,
            Units.degreesToRadians(0), // The angle at which the arm starts at during simulation.
            0.0,
            0.0 
        );

    public Arm(){
        brakeMode.setDefaultOption("Brake", true);
        brakeMode.addOption("Coast", false);

        SmartDashboard.putData("Arm idle mode", brakeMode);

        rotatorConfig
            .idleMode(IdleMode.kBrake)
            .inverted(true)

        .encoder
            .positionConversionFactor(OperatorConstants.m_armConversionFactor)
            .velocityConversionFactor(OperatorConstants.m_armConversionFactor/60);
    
        m_armRotator.configure(rotatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_controller.setTolerance(4);

        setPosition(0.0);
    }

    public double getMeasurement(){
        return encoder.getPosition();
    }

    public double getSimAngle() {
        return m_armSim.getAngleRads();
    }

    public void setPosition(double position){
        goal = position;
    }
    public Command runArm(){
        return run(()->{m_armRotator.set(m_controller.calculate(getMeasurement(), goal));});
    }
    
    public void runMotor(double d){
        m_armRotator.set(d);
    }

    public BooleanSupplier atGoal(){
        return () -> (m_controller.atSetpoint());
    }

    public void stop(){
        m_armRotator.set(0.0);
    }

    public void zero(){
        encoder.setPosition(0);
    }

    public void coastMode(){
        rotatorConfig.idleMode(IdleMode.kCoast);
        m_armRotator.configure(rotatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void brakeMode(){
        rotatorConfig.idleMode(IdleMode.kBrake);
        m_armRotator.configure(rotatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Arm Position", getMeasurement());
        SmartDashboard.putData("Arm Pid", m_controller);
        SmartDashboard.putNumber("Arm Output", m_controller.calculate(getMeasurement()));
        SmartDashboard.putNumber("Arm Goal", goal);
        SmartDashboard.putBoolean("Arm At Goal", m_controller.atSetpoint());
        SmartDashboard.putNumber("Abolute Encoder", rotatorAbsoluteEncoder.getPosition());
    }

    @Override
    public void simulationPeriodic(){
        m_armSim.setInput(m_armMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

        SmartDashboard.putNumber("Arm Motor Output Sim", m_armMotorSim.getAppliedOutput());

        m_armSim.update(0.02);

        m_armMotorSim.iterate(
            Units.radiansPerSecondToRotationsPerMinute(m_armSim.getVelocityRadPerSec()), 
            RoboRioSim.getVInVoltage(), 
            0.02
        );

        encoder.setPosition(
            Units.radiansToDegrees(
                m_armSim.getAngleRads()
            )
        );

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(
                m_armSim.getCurrentDrawAmps()
            )
        );

        // Pose3d logging for visualizing mechanisms in 3d
        DogLog.log("ZeroedComponentPoses", new Pose3d[] {new Pose3d()});
        DogLog.log("ArmPose3d", new Pose3d[] {
            new Pose3d(
            0, 
            -0.0047752, 
            (elevator.getElevatorDistance() * 2) + 0.2381504, 
            new Rotation3d(
                0, 
                -m_armSim.getAngleRads() + Units.degreesToRadians(90 - 54.133228), 
                0
            ))
        });
    }
}