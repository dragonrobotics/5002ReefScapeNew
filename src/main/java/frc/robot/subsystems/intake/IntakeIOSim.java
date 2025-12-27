package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;

import org.ironmaple.simulation.IntakeSimulation;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.MapleSimSwerveDrivetrain;

public class IntakeIOSim implements IntakeIO {
    private final DCMotor genericSimMotor = DCMotor.getNEO(1);

    private final SparkMaxSim intakeSimMotor = new SparkMaxSim(
        intakeMotor,
        genericSimMotor
    );

    private final FlywheelSim intakeWheelSimulation = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            genericSimMotor, 
            SingleJointedArmSim.estimateMOI(
                Units.inchesToMeters(2),
                Units.lbsToKilograms( 0.269 * 4)
            ), 
            1
        ), 
        genericSimMotor, 
        0
    );

    private final IntakeSimulation intakeSimulation;

    public IntakeIOSim(MapleSimSwerveDrivetrain simulatedSwerveDrive) {
        intakeSimulation = IntakeSimulation.InTheFrameIntake(
            "Coral", 
            simulatedSwerveDrive.mapleSimDrive,
            Inches.of(4),
            IntakeSimulation.IntakeSide.FRONT,
            1
        );
    }

    @Override
    public void updateInputs(IntakeInputs intakeInputs) {
        intakeWheelSimulation.setInput(intakeSimMotor.getAppliedOutput() * RoboRioSim.getVInVoltage());

        intakeWheelSimulation.update(Constants.simUpdateLatency);

        intakeSimMotor.iterate(
            intakeWheelSimulation.getAngularVelocityRPM(),
            RoboRioSim.getVInVoltage(),
            Constants.simUpdateLatency   
        );

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(
                intakeWheelSimulation.getCurrentDrawAmps()
            )
        );

        intakeInputs.motorCurrent = intakeSimMotor.getMotorCurrent();
        intakeInputs.intakeHasGamePiece = intakeSimMotor.getMotorCurrent() >= 30;
    }

    @Override
    public void runIntake(double speed) {
        intakeSimMotor.setAppliedOutput(speed);
    }

    @Override
    public void stop() {
        intakeSimMotor.setAppliedOutput(0);
    }
}
