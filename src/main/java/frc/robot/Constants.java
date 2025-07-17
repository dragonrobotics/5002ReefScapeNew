// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.Thread.State;
import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.Dictionary;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

 

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;

    //elevator Motor controllers
    public final static int m_elevator = 2;
    public final static int m_elevatorFollower = 6;
    
    public final static double elvatorConversionFactor = 0.59635;

    //Arm Motor controllers
    public final static int m_armRotator = 15;
    public final static int m_armShooter = 1;

    public final static double m_armConversionFactor = 2.5697;

    //Climber Motor Controllers
    public final static int m_climber = 4;

    //AutoAIm:
    public static final String cameraName = "photonvision";
    public static final double distanceToTag = 16 * 2.54;

    public static final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  

    //Arm Constants
    public final int beamBreakId = 0;

    //Vision
  }

  public static final class RobotConstants{

    public static final double elevatorHeight = 28.0;
  }

  public static final Map<String, Pose2d[]> autoAlignSide;

  public static final Pose2d[] blueAutoPoses;
  public static final Pose2d[] redAutoPoses;

  public static final Pose2d[] blueIntakePoses;
  public static final Pose2d[] redIntakePoses;
  


  static {
    autoAlignSide = new HashMap<String, Pose2d[]>();

    blueAutoPoses = new Pose2d[6];
    redAutoPoses = new Pose2d[6];

    blueIntakePoses = new Pose2d[2];
    redIntakePoses = new Pose2d[2];

    blueAutoPoses[0] = (OperatorConstants.layout.getTagPose(21).get().toPose2d());
    blueAutoPoses[1] = (OperatorConstants.layout.getTagPose(22).get().toPose2d());
    blueAutoPoses[2] = (OperatorConstants.layout.getTagPose(17).get().toPose2d());
    blueAutoPoses[3] = (OperatorConstants.layout.getTagPose(18).get().toPose2d());
    blueAutoPoses[4] = (OperatorConstants.layout.getTagPose(19).get().toPose2d());
    blueAutoPoses[5] = (OperatorConstants.layout.getTagPose(20).get().toPose2d());

    redAutoPoses[0] = (OperatorConstants.layout.getTagPose(10).get().toPose2d());
    redAutoPoses[1] = (OperatorConstants.layout.getTagPose(9).get().toPose2d());
    redAutoPoses[2] = (OperatorConstants.layout.getTagPose(8).get().toPose2d());
    redAutoPoses[3] = (OperatorConstants.layout.getTagPose(7).get().toPose2d());
    redAutoPoses[4] = (OperatorConstants.layout.getTagPose(6).get().toPose2d());
    redAutoPoses[5] = (OperatorConstants.layout.getTagPose(11).get().toPose2d());

    blueIntakePoses[0] = (OperatorConstants.layout.getTagPose(12).get().toPose2d());
    blueIntakePoses[1] = (OperatorConstants.layout.getTagPose(13).get().toPose2d());

    redIntakePoses[0]= (OperatorConstants.layout.getTagPose(1).get().toPose2d());
    redIntakePoses[1]= (OperatorConstants.layout.getTagPose(2).get().toPose2d());

    autoAlignSide.put("Blue", blueAutoPoses);
    autoAlignSide.put("Red", redAutoPoses);
    
    
  }
}
