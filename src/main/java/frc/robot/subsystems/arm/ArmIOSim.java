package frc.robot.subsystems.arm;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class ArmIOSim implements ArmIO {
    private final DCMotor genericSimMotor = DCMotor.getNEO(1);
    
    private final SparkMaxSim armSimMotor = new SparkMaxSim(ArmIOReal.armMotor, genericSimMotor);

    private final SparkRelativeEncoderSim armSimEncoder = armSimMotor.getRelativeEncoderSim();

    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
        genericSimMotor,
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
    
    @Override
    public void updateInputs(ArmInputs armInputs) {
        armSim.setInput(armSimMotor.getAppliedOutput() * RoboRioSim.getVInVoltage());

        armSim.update(Constants.simUpdateLatency);

        armSimMotor.iterate(
            armSim.getVelocityRadPerSec(),
            RoboRioSim.getVInVoltage(),
            Constants.simUpdateLatency
        );

        armSimEncoder.setPosition(armSim.getAngleRads());
        armSimEncoder.setVelocity(armSim.getVelocityRadPerSec());

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(
                armSim.getCurrentDrawAmps()
            )
        );

        armInputs.angle = armSimEncoder.getPosition();
        armInputs.absAngle = armSimEncoder.getVelocity();

        armInputs.velocity = armSim.getVelocityRadPerSec();

        armInputs.motorCurrent = armSimMotor.getMotorCurrent();
    }

    @Override
    public void runArm(double speed) {
        armSimMotor.setAppliedOutput(speed);
    }

    @Override
    public void stopArm() {
        armSimMotor.setAppliedOutput(0);
    }

    @Override
    public void resetEncoder() {
        armSim.setState(0, 0);
    }
}
