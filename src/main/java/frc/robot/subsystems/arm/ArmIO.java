package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.CAN_IDs;

public interface ArmIO {
    static final SparkMax armMotor = new SparkMax(CAN_IDs.armMotor, MotorType.kBrushless);
    
    class ArmInputs {
        double angle = 0;
        double absAngle = 0;

        double velocity = 0;

        double motorCurrent = 0;
    }
    
    public void updateInputs(ArmInputs armInputs);

    public default void runArm(double speed) {}

    public default void stopArm() {}

    public default void resetEncoder() {}
}