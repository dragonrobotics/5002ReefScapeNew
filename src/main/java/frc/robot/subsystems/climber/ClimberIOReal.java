package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ClimberIOReal implements ClimberIO {
    private final RelativeEncoder climberEncoder = climberMotor.getEncoder();

    private final SparkMaxConfig climberConfig = new SparkMaxConfig();

    public ClimberIOReal() {
        climberConfig
            .idleMode(IdleMode.kBrake);

        climberMotor.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ClimberInputs climberInputs) {
        climberInputs.angle = climberEncoder.getPosition();
        climberInputs.velocity = climberEncoder.getVelocity();

        climberInputs.motorCurrent = climberMotor.getOutputCurrent();
    }

    @Override
    public void runClimber(double speed) {
        climberMotor.set(speed);
    }

    @Override
    public void stopClimber() {
        climberMotor.stopMotor();
    }

    @Override
    public void resetEncoder() {
        climberEncoder.setPosition(0);
    }
}
