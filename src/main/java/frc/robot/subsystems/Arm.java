package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.EncoderConfigAccessor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.OperatorConstants;

public class Arm extends SubsystemBase {
    
    final SparkMax m_armRotator = new SparkMax(OperatorConstants.m_armRotator, MotorType.kBrushless);

    SparkMaxConfig rotatorConfig = new SparkMaxConfig();

    final RelativeEncoder encoder = m_armRotator.getEncoder();
    final AbsoluteEncoder rotatorAbsoluteEncoder = m_armRotator.getAbsoluteEncoder();

    PIDController m_controller = new PIDController(0.12, 0.02, 0);
    public double goal = 0.0;
    double factor = 0.0;

    public SendableChooser<Boolean> brakeMode = new SendableChooser<Boolean>();

    private Intake intake = RobotContainer.intake;
    private Elevator elevator = RobotContainer.elevator;

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

        m_controller.reset();

        m_controller.setTolerance(1);
        setPosition(0.0);
    }

    public double getMeasurement(){
        return encoder.getPosition();
    }

    public void setPosition(double position){
        m_controller.reset();
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
        SmartDashboard.putBoolean("Arm At Goal", m_controller.atSetpoint());
    }
}