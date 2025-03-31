package frc.robot.subsystems;

// import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
// import static edu.wpi.first.wpilibj2.command.Commands.sequence;
// import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

// import java.lang.ModuleLayer.Controller;
import java.util.function.BooleanSupplier;

// import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
//Hello
import edu.wpi.first.math.controller.ElevatorFeedforward;
// import edu.wpi.first.math.controller.PIDController;
import frc.math.PIDEController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
// import edu.wpi.first.math.trajectory.ExponentialProfile.State;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;


public class Elevator extends SubsystemBase{

    //goes from 0 to 27.5

    final SparkMax m_elevator = new SparkMax(OperatorConstants.m_elevator, MotorType.kBrushless);
    final SparkMax m_follower = new SparkMax(OperatorConstants.m_elevatorFollower, MotorType.kBrushless);

    SparkMaxConfig mainConfig = new SparkMaxConfig();
    SparkMaxConfig followerConfig = new SparkMaxConfig();

    final RelativeEncoder encoder = m_elevator.getEncoder();
    
    PIDEController controller = new PIDEController(0.7, 0.1, 0.05);

    ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0.25, 0.1);

    Double factor = 0.0;
    Double goal = 0.0;
    
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
    //    SmartDashboard.putData("Elevator FeedForward", (Sendable) feedforward);
        SmartDashboard.putData("Elevator PID", controller);
    }

}
