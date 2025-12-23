package frc.robot.subsystems.arm;

public interface ArmIO {

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