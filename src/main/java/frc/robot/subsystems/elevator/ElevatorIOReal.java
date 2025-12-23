package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.EncoderFactors;

public class ElevatorIOReal implements ElevatorIO {
    private final RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

    private final SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    private final SparkMaxConfig followerConfig = new SparkMaxConfig();

    public ElevatorIOReal() {
        elevatorConfig
            .idleMode(IdleMode.kBrake)
        .encoder
            .positionConversionFactor(EncoderFactors.elevatorFactor)
            .velocityConversionFactor(EncoderFactors.elevatorFactor/60);

        followerConfig
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .follow(elevatorMotor);

        elevatorMotor.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorFollower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ElevatorInputs elevatorInputs) {
        elevatorInputs.height = elevatorEncoder.getPosition();
        elevatorInputs.velocity = elevatorEncoder.getVelocity();

        elevatorInputs.motorCurrent = elevatorMotor.getOutputCurrent();
    }

    @Override
    public void runElevator(double speed) {
        elevatorMotor.set(speed);
    }

    @Override
    public void stopElevator() {
        elevatorMotor.stopMotor();
    }

    @Override
    public void resetEncoder() {
        elevatorEncoder.setPosition(0);
    }
}
