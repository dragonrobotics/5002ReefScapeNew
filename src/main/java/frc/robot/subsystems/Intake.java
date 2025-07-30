package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.BooleanSupplier;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.OperatorConstants;

@Logged
public class Intake extends SubsystemBase {
    private double robotPoseX;
    private double robotPoseY;
    private double robotRotation;

    // References to 
    private Elevator elevator = RobotContainer.elevator;;
    private Arm arm = RobotContainer.arm;

    final SparkMax m_shooter = new SparkMax(OperatorConstants.m_armShooter, MotorType.kBrushless);

    SparkMaxConfig shooterConfig = new SparkMaxConfig();

    /*
     * Simulates intake functionality in AdvantageScope
     */

    // Needed to simulate the intake amps in simulation.
    private final DCMotor m_intakeGearBox = DCMotor.getNEO(1);
    private final SparkMaxSim m_shooterMotorSim = new SparkMaxSim(m_shooter, m_intakeGearBox);

    @NotLogged
    private final LinearSystem<N1,N1,N1> m_plant = LinearSystemId.createFlywheelSystem(
        m_intakeGearBox, 
        0.1, 
        1
    );

    private final FlywheelSim m_intakeSim = new FlywheelSim(
        m_plant, 
        m_intakeGearBox, 
    0
    );

    public Intake(){
        shooterConfig
            .idleMode(IdleMode.kBrake);

        m_shooter.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public void runIntake(double speed){
        m_shooter.setVoltage(speed);
    }

    public void stopIntake(){
        m_shooter.stopMotor();
    }

    public double getAmps(){
        return m_shooter.getOutputCurrent();
    }

    public BooleanSupplier gotCoral(){
        return ()->Math.abs(getAmps()) >= 30;
    }

    public void getRobotTransformations(double[] robotPose){
        robotPoseX = robotPose[0];
        robotPoseY = robotPose[1];
        robotRotation = robotPose[2];
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Intake Amps", getAmps());
    }

    @Override
    public void simulationPeriodic() {
        DogLog.log("ZeroedComponentPoses", new Pose3d[] {new Pose3d()});
        DogLog.log("Intake Coral Pose", new Pose3d[] {
            new Pose3d(
                robotPoseX, 
                robotPoseY, 
                0, 
                new Rotation3d(
                    0, 
                    0, 
                    Units.degreesToRadians(robotRotation+180)
                )
            ).transformBy(
                new Transform3d(
                    0, 
                    0.0047752, 
                    (elevator.getElevatorDistance() * 2) + 0.2381504, 
                    new Rotation3d(
                        0, 
                        arm.getSimAngle() + Units.degreesToRadians(54.133228-90),
                        0
                    )
                )
            ).transformBy(
                new Transform3d(
                    0.2273, 
                    -0.0619, 
                    0.3990, 
                    new Rotation3d(
                        0, 
                        Units.degreesToRadians(270 - 54.133228), 
                        0
                    )
                )
            )
        });
    }    
}
