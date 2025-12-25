package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class IntakeIOReal implements IntakeIO {
    private final SparkMaxConfig intakeConfig = new SparkMaxConfig();

    public IntakeIOReal() {
        intakeConfig
            .idleMode(IdleMode.kBrake);

        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(IntakeInputs intakeInputs) {
        intakeInputs.motorCurrent = intakeMotor.getOutputCurrent();
        intakeInputs.intakeHasGamePiece = intakeMotor.getOutputCurrent() >= 30;
    }

    @Override
    public void runIntake(double speed) {
        intakeMotor.set(speed);
    }

    @Override
    public void stop() {
        intakeMotor.stopMotor();
    }
}