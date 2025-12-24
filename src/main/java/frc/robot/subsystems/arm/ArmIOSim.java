package frc.robot.subsystems.arm;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class ArmIOSim implements ArmIO {
    private final DCMotor genericSimMotor = DCMotor.getNEO(1);
    
    private final SparkMaxSim armSimMotor = new SparkMaxSim(armMotor, genericSimMotor);

    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
        genericSimMotor,
        3,
        SingleJointedArmSim.estimateMOI(
            Units.inchesToMeters(23.092), 
            Units.lbsToKilograms(2.5917853)
        ),
        Units.inchesToMeters(23.092),
        -2 * Math.PI,
        2 * Math.PI,
        true,
        0,
        0.0,
        0.0 
    );
    
    @Override
    public void updateInputs(ArmInputs armInputs) {
        armSim.setInput(armSimMotor.getAppliedOutput() * RoboRioSim.getVInVoltage());

        armSim.update(Constants.simUpdateLatency);

        armSimMotor.iterate(
            Units.radiansPerSecondToRotationsPerMinute(
                armSim.getVelocityRadPerSec()
            ),
            RoboRioSim.getVInVoltage(),
            Constants.simUpdateLatency
        );

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(
                armSim.getCurrentDrawAmps()
            )
        );

        armInputs.angle = Units.radiansToDegrees(armSim.getAngleRads());
        armInputs.absAngle = Units.radiansToDegrees(armSim.getAngleRads());

        armInputs.velocity = Units.radiansToDegrees(armSim.getVelocityRadPerSec());

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
