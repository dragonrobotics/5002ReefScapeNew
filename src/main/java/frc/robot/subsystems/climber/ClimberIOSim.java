package frc.robot.subsystems.climber;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class ClimberIOSim implements ClimberIO {
    private final DCMotor genericSimMotor = DCMotor.getNEO(1);

    private final SparkMaxSim climberSimMotor = new SparkMaxSim(climberMotor, genericSimMotor);

    private final SparkRelativeEncoderSim climberSimEncoder = climberSimMotor.getRelativeEncoderSim();

    private final SingleJointedArmSim climberSim = new SingleJointedArmSim(
        DCMotor.getNEO(1),
        1, 
        SingleJointedArmSim.estimateMOI(
            Units.inchesToMeters(8), 
            Units.lbsToKilograms(3.4874496)
            ),
        Units.inchesToMeters(18),
        Units.degreesToRadians(0),
        Units.degreesToRadians(170),
        true,
        Units.degreesToRadians(0), 
        0.0,
        0.0
    );

    @Override
    public void updateInputs(ClimberInputs climberInputs) {
        climberSim.setInput(climberSimMotor.getAppliedOutput() * RoboRioSim.getVInVoltage());

        climberSim.update(Constants.simUpdateLatency);

        climberSimMotor.iterate(
            climberSim.getVelocityRadPerSec(),
            RoboRioSim.getVInVoltage(),
            Constants.simUpdateLatency
        );

        climberSimEncoder.iterate(
            climberSim.getVelocityRadPerSec(),
            Constants.simUpdateLatency   
        );

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(
                climberSim.getCurrentDrawAmps()
            )
        );

        climberInputs.angle = climberSimEncoder.getPosition();
        climberInputs.velocity = climberSimEncoder.getVelocity();

        climberInputs.motorCurrent = climberSimMotor.getMotorCurrent();
    }

    @Override
    public void runClimber(double speed) {
        climberSimMotor.setAppliedOutput(speed);
    }

    @Override
    public void stopClimber() {
        climberSimMotor.setAppliedOutput(0);
    }

    @Override
    public void resetEncoder() {
        climberSimEncoder.setPosition(0);
    }
}
