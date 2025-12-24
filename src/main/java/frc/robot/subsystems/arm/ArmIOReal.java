package frc.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ArmIOReal implements ArmIO {
    private final RelativeEncoder armRelEncoder = armMotor.getEncoder();
    private final AbsoluteEncoder armAbsEncoder = armMotor.getAbsoluteEncoder();

    private final SparkMaxConfig armConfig = new SparkMaxConfig();

    public ArmIOReal() {
        armConfig
            .idleMode(IdleMode.kBrake);

        armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ArmInputs armInputs) {
        armInputs.angle = armRelEncoder.getPosition();
        armInputs.absAngle = armAbsEncoder.getPosition();

        armInputs.velocity = armRelEncoder.getVelocity();

        armInputs.motorCurrent = armMotor.getOutputCurrent();
    }

    @Override
    public void runArm(double speed) {
        armMotor.set(speed);
    }

    @Override
    public void stopArm() {
        armMotor.stopMotor();
    }

    @Override
    public void resetEncoder() {
        armRelEncoder.setPosition(0);
    }
}
