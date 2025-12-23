package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.CAN_IDs;

public interface ElevatorIO {
    static final SparkMax elevatorMotor = new SparkMax(CAN_IDs.elevatorMotor, MotorType.kBrushless);
    static final SparkMax elevatorFollower = new SparkMax(CAN_IDs.elevatorFollower, MotorType.kBrushless);

    public class ElevatorInputs {
        public double height = 0;
        double velocity = 0;

        double motorCurrent = 0;
    }

    public void updateInputs(ElevatorInputs elevatorInputs);

    public default void runElevator(double speed) {}

    public default void stopElevator() {}

    public default void resetEncoder() {}
}
