package frc.robot.subsystems.elevator;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO {
    private final DCMotor genericSimMotors = DCMotor.getNEO(2);

    private final SparkMaxSim elevatorSimMotor = new SparkMaxSim(elevatorMotor, genericSimMotors);

    private final ElevatorSim elevatorSim = new ElevatorSim(
        genericSimMotors,
        3,
        Units.lbsToKilograms(45),
        Units.inchesToMeters(0.75),
        Units.inchesToMeters(0),
        Units.inchesToMeters(28),
        true,
        Units.inchesToMeters(0),
        0.0,
        0.0
    );

    @Override
    public void updateInputs(ElevatorInputs elevatorInputs) {
        elevatorSim.setInput(elevatorSimMotor.getAppliedOutput() * RoboRioSim.getVInVoltage());
        elevatorSim.update(Constants.simUpdateLatency);

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(
                elevatorSim.getCurrentDrawAmps()
            )
        );

        elevatorInputs.height = Units.metersToInches(
            elevatorSim.getPositionMeters()
        );

        elevatorInputs.velocity = Units.metersToInches(
            elevatorSim.getVelocityMetersPerSecond()
        );

        elevatorInputs.motorCurrent = elevatorSim.getCurrentDrawAmps();
    }

    @Override
    public void runElevator(double speed) {
        elevatorSimMotor.setAppliedOutput(speed);
    }

    @Override
    public void runElevatorVolts(double volts) {
        elevatorSimMotor.setAppliedOutput(
            volts / RoboRioSim.getVInVoltage()
        );
        
    }

    @Override
    public void stopElevator() {
        elevatorSimMotor.setAppliedOutput(0);
    }

    @Override
    public void resetEncoder() {
        elevatorSim.setState(0, 0);
    }
}
