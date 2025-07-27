package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import dev.doglog.DogLog;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;
@Logged
public class Climber extends SubsystemBase {

    final SparkMax m_climber = new SparkMax(OperatorConstants.m_climber, MotorType.kBrushless);

    final RelativeEncoder encoder = m_climber.getEncoder();

    SparkMaxConfig climberConfig = new SparkMaxConfig();

    /*
     * Simulates the climber for AdvantageScope
     */

    private final DCMotor m_climberGearBoxSim = DCMotor.getNEO(1);

    private final SparkMaxSim m_climberMotorSim = new SparkMaxSim(m_climber, m_climberGearBoxSim);
    
    private final SingleJointedArmSim m_climberSim = 
        new SingleJointedArmSim(
            m_climberGearBoxSim,
            1, // Represents a 3:1 gear ratio between the 20T/60T gears
            SingleJointedArmSim.estimateMOI(
                Units.inchesToMeters(8), 
                Units.lbsToKilograms(3.4874496)
                ),
            Units.inchesToMeters(18), // The length of our climber
            Units.degreesToRadians(0), // The lowest angle our climber can aim towards.
            Units.degreesToRadians(170), // Limits the climber to a vertical angle.
            true,
            Units.degreesToRadians(0), // The angle at which the climber starts at during simulation.
            0.0,
            0.0
        );

    public Climber(){

        climberConfig
            .idleMode(IdleMode.kBrake);

        m_climber.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
                    0,
                    -0.31115,
                    0.3651504,
                    new Rotation3d(
                        m_climberSim.getAngleRads(), 
                        0, 
                        0))
            }
        );
    }
}