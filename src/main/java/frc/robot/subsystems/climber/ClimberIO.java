package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.CAN_IDs;

public interface ClimberIO {
    static SparkMax climberMotor = new SparkMax(CAN_IDs.climberMotor, MotorType.kBrushless);
    
    class ClimberInputs {
        double angle = 0;
        double velocity = 0;

        double motorCurrent = 0;
    }

    public void updateInputs(ClimberInputs climberInputs);

    public default void runClimber(double speed) {}

    public default void stopClimber() {}

    public default void resetEncoder() {}
}