package frc.robot.subsystems.intake;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIO.IntakeInputs;

public class Intake extends SubsystemBase {
    private final IntakeIO intakeIO;
    private final IntakeInputs intakeInputs = new IntakeInputs();

    public Intake(IntakeIO intakeIO) {
        this.intakeIO = intakeIO;
    }

    public Command runIntake(double speed) {
        return run(
            () -> intakeIO.runIntake(speed)
        );
    }

    public Command stopIntake() {
        return run(
            () -> intakeIO.stopIntake()
        );
    }

    public Command intakeCoral() {
        return runIntake(4);
    }

    public Command shootCoral() {
        return runIntake(-4);
    }

    public Command intakeCoralOnce() {
        return sequence(
            runIntake(8),
            waitSeconds(2),
            stopIntake()
        );
    }

    public Command shootCoralOnce() {
        return sequence(
            runIntake(-8),
            waitSeconds(2),
            stopIntake()
        );
    }

    @Override
    public void periodic() {
        intakeIO.updateInputs(intakeInputs);

        SmartDashboard.putNumber("Intake Motor Amps", intakeInputs.motorCurrent);
        SmartDashboard.putBoolean("Intake Has Coral?", intakeInputs.intakeHasGamePiece);
    }
}
