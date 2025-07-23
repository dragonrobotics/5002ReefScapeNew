package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import dev.doglog.DogLog;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Telemetry;
import frc.robot.Constants.OperatorConstants;
@Logged
public class Climber extends SubsystemBase {

    final SparkMax m_climber = new SparkMax(OperatorConstants.m_climber, MotorType.kBrushless);

    final RelativeEncoder encoder = m_climber.getEncoder();

    SparkMaxConfig climberConfig = new SparkMaxConfig();

    // Simulates the climber using a virtual motor

    private final DCMotor m_climberGearBoxSim = DCMotor.getNEO(1);

    private final SparkMaxSim m_climberMotorSim = new SparkMaxSim(m_climber, m_climberGearBoxSim);
    
    private final SingleJointedArmSim m_climberSim = 
        new SingleJointedArmSim(
            m_climberGearBoxSim,
            1, // Represents a 3:1 gear ratio between the 20T/60T gears
            SingleJointedArmSim.estimateMOI(Units.inchesToMeters(8), 1.5),
            Units.inchesToMeters(18), // The length of our climber
            Units.degreesToRadians(46.828525), // The lowest angle our climber can aim towards.
            Units.degreesToRadians(270), // Limits the climber to a vertical angle.
            true,
            Units.degreesToRadians(46.828525), // The angle at which the climber starts at during simulation.
            0.0,
            0.0 // Add noise with a std-dev of 1 tick
        );

    // Mechanism2d code to visualize the climber in AdvantageScope

    public final Mechanism2d climberMech2d = new Mechanism2d(1, 1);
    private final MechanismRoot2d climberPivot =  climberMech2d.getRoot(
        "climberPivot", 
        1.18, 
        Units.inchesToMeters(1.876)
    );

    // Adds the metal beams that hold the pivoting part of the climber in place.
    private final MechanismLigament2d m_climberSupport = climberPivot.append(
        new MechanismLigament2d(
            "Climber Support", 
            Units.inchesToMeters(13.5), 
            90)
    );

    // Adds the rotating piece of the climber.
    private final MechanismLigament2d m_climberRotating = m_climberSupport.append(
        new MechanismLigament2d(
            "Climber Rotating Piece", 
            Units.inchesToMeters(8), 
            Units.radiansToDegrees(m_climberSim.getAngleRads()))
    );


    private final MechanismLigament2d m_climberExtension = m_climberRotating.append(
        new MechanismLigament2d(
            "Climber Extension", 
            Units.inchesToMeters(7.084000), 
            - Units.radiansToDegrees(m_climberSim.getAngleRads()) + 90
        )
    );

    public Climber(){

        climberConfig
            .idleMode(IdleMode.kBrake);

        m_climber.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_climberSupport.setLineWeight(2.5);
        m_climberRotating.setLineWeight(2);
        m_climberExtension.setLineWeight(2);

        m_climberRotating.setColor(new Color8Bit(Color.kAqua));
        m_climberExtension.setColor(new Color8Bit(Color.kAqua));


        SmartDashboard.putData("Climber Sim", climberMech2d);
    }

    public void runClimber(double speed){
        m_climber.setVoltage(speed);
    }

    public void stop(){
        m_climber.stopMotor();
    }

    @Override
    public void simulationPeriodic() {
        // Simulates the idleMode from the real climber motor
        if (Math.abs(m_climberMotorSim.getAppliedOutput()) < 0.02) {
            m_climberSim.setInput(0); // Stops the virtual motor
        } 

        m_climberSim.setInput(m_climberMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

        m_climberSim.update(0.02);

        m_climberMotorSim.iterate(
            Units.radiansPerSecondToRotationsPerMinute(m_climberSim.getVelocityRadPerSec()), 
            RoboRioSim.getVInVoltage(), 
            0.02
        );

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_climberSim.getCurrentDrawAmps()));


                // Pose3d logging for visualizing mechanisms in 3d
        DogLog.log("ZeroedComponentPoses", new Pose3d[] {new Pose3d()});
        DogLog.log(
            "ClimberPose3d", 
            new Pose3d[] {
                new Pose3d(
                    -0.3048,
                    0,
                    0.3651504,
                    new Rotation3d(
                        0, 
                        - m_climberSim.getAngleRads() + Units.degreesToRadians(45), 
                        0))
            });

        
        m_climberRotating.setAngle(Units.radiansToDegrees(m_climberSim.getAngleRads()) - 90);      
    }
}