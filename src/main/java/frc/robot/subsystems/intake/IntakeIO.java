package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.CAN_IDs;

public interface IntakeIO {
    static SparkMax intakeMotor = new SparkMax(CAN_IDs.intakeMotor, MotorType.kBrushless);

    class IntakeInputs {
        double motorCurrent = 0;

        boolean intakeHasGamePiece = false;
    }

    public void updateInputs(IntakeInputs intakeInputs);

    public default void runIntake(double speed) {}

    public default void stopIntake() {}
}
