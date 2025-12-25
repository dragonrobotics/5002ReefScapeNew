package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;

import org.ironmaple.simulation.IntakeSimulation;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.swerve.MapleSimSwerveDrivetrain;

public class IntakeIOSim implements IntakeIO {
    private final SparkMaxSim intakeSimMotor = new SparkMaxSim(
        intakeMotor,
        DCMotor.getNEO(1)
    );

    private final IntakeSimulation intakeSimulation;

    public IntakeIOSim(MapleSimSwerveDrivetrain simulatedSwerveDrive) {
        intakeSimulation = IntakeSimulation.InTheFrameIntake(
            "Coral", 
            simulatedSwerveDrive.mapleSimDrive,
            Meters.of(0.5),
            IntakeSimulation.IntakeSide.FRONT,
            1
        );
    }

    @Override
    public void updateInputs(IntakeInputs intakeInputs) {
        intakeInputs.motorCurrent = intakeSimMotor.getMotorCurrent();
        intakeInputs.intakeHasGamePiece = intakeSimulation.getGamePiecesAmount() != 0;
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
