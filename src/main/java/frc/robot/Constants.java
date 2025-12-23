package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;

public class Constants {
    public static final int xboxControllerPort = 0;
    public static final int buttonBoardPort = 1;

    public static final double simUpdateLatency = 0.02;

    public static final double controllerDeadband = 0.1;

    public static class CAN_IDs {
        public static final int armMotor = 15;

        public static final int climberMotor = 4;

        public static final int elevatorMotor = 2;
        public static final int elevatorFollower = 6;

        public static final int intakeMotor = 1;
    }

    public static class EncoderFactors {
        public static final double armFactor = 2.5697;
        public static final double elevatorFactor = 0.59635;
    }

    public static class PID {
        public static class Arm {
            public static final double kP = 0.1;
            public static final double kI = 0.0;
            public static final double kD = 0.0;

            public static final double maxVelocity = 99;
            public static final double maxAcceleration = 99;

            public static final double tolerance = 2;
        }

        public static class Elevator {
            public static final double kP = 0.8;
            public static final double kI = 0;
            public static final double KD = 0.05;

            public static final double maxVelocity = 5;
            public static final double maxAcceleration = 2;

            public static final double tolerance = 0.25;
        }       
    }

    public static class InitialSimPose {
        public static final double xPosition = 7.5;
        public static final double yPosition = 4.025;

        public static final Angle angle = Degrees.of(180);
    }

    public static class PhotonVision {
        public static final String cameraName = "photonvision";

        public static final boolean simRawSteamEnabled = true;
        public static final boolean simProcessedSteamEnabled = true;

        public static final double simCameraFPS = 60;
        public static final int simCameraResWidth = 1920;
        public static final int simCameraResHeight = 1080;
    }

    public static class AprilTagPoses {
        public static final AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        public static final Map<String, Pose2d[]> autoAlignSide = new HashMap<String, Pose2d[]>();

        public static final Pose2d[] blueAutoPoses = new Pose2d[6];
        public static final Pose2d[] redAutoPoses = new Pose2d[6];

        public static final Pose2d[] blueIntakePoses = new Pose2d[2];
        public static final Pose2d[] redIntakePoses = new Pose2d[2];

        static {
            blueAutoPoses[0] = (tagLayout.getTagPose(21).get().toPose2d());
            blueAutoPoses[1] = (tagLayout.getTagPose(22).get().toPose2d());
            blueAutoPoses[2] = (tagLayout.getTagPose(17).get().toPose2d());
            blueAutoPoses[3] = (tagLayout.getTagPose(18).get().toPose2d());
            blueAutoPoses[4] = (tagLayout.getTagPose(19).get().toPose2d());
            blueAutoPoses[5] = (tagLayout.getTagPose(20).get().toPose2d());
        
            redAutoPoses[0] = (tagLayout.getTagPose(10).get().toPose2d());
            redAutoPoses[1] = (tagLayout.getTagPose(9).get().toPose2d());
            redAutoPoses[2] = (tagLayout.getTagPose(8).get().toPose2d());
            redAutoPoses[3] = (tagLayout.getTagPose(7).get().toPose2d());
            redAutoPoses[4] = (tagLayout.getTagPose(6).get().toPose2d());
            redAutoPoses[5] = (tagLayout.getTagPose(11).get().toPose2d());
        
            blueIntakePoses[0] = (tagLayout.getTagPose(12).get().toPose2d());
            blueIntakePoses[1] = (tagLayout.getTagPose(13).get().toPose2d());
        
            redIntakePoses[0]= (tagLayout.getTagPose(1).get().toPose2d());
            redIntakePoses[1]= (tagLayout.getTagPose(2).get().toPose2d());
        
            autoAlignSide.put("Blue", blueAutoPoses);
            autoAlignSide.put("Red", redAutoPoses);
        }
    }
}
