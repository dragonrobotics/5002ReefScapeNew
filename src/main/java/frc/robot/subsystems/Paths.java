package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.none;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.swerve.jni.SwerveJNI.DriveState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Paths extends SubsystemBase {
    private CommandSwerveDrivetrain drivetrain;
    public NetworkTable buttonTable;
    public Command currentCommand;
    private int indexNum;

    private Boolean[] group1left;
    private Boolean[] group2left;
    private Boolean[] group3left;
    private Boolean[] group4left;
    private Boolean[] group5left;
    private Boolean[] group6left;

    private Boolean[] group1right;
    private Boolean[] group2right;
    private Boolean[] group3right;
    private Boolean[] group4right;
    private Boolean[] group5right;
    private Boolean[] group6right;

    private Boolean[][] lefties;
    private Boolean[][] righties;

    private PathConstraints contraints; 
    public Paths(CommandSwerveDrivetrain driveTrain){
        
        group1left = new Boolean[3];
        group1right = new Boolean[3];

        group2left = new Boolean[3];
        group2right = new Boolean[3];

        group3left = new Boolean[3];
        group3right = new Boolean[3];

        group4left = new Boolean[3];
        group4right = new Boolean[3];

        group5left = new Boolean[3];
        group5right = new Boolean[3];

        group6left = new Boolean[3];
        group6right = new Boolean[3];

        lefties = new Boolean[6][3];
        righties = new Boolean[6][3];

        buttonTable = NetworkTableInstance.getDefault().getTable("VirtualButtonBoard");
        drivetrain = driveTrain;
        contraints = new PathConstraints(
            2.0, // 3 m/s
            4.0,
            Units.degreesToRadians(540), 
            Units.degreesToRadians(720));
    }

    public Command pathTo(Pose2d pose){
        Command pathFindingCommand;
        pathFindingCommand = AutoBuilder.pathfindToPose(
                pose,
                contraints,
                0.0
                );
        return pathFindingCommand;
    }

    public Pose2d transformOffset(
        Pose2d pose,
        double inchesForward,
        double inchesSide,
        double degreesOffset // heading change relative to current heading
) {
    double metersForward = inchesForward * 0.0254;
    double metersSide = inchesSide * 0.0254;

    Translation2d offset = new Translation2d(metersForward, metersSide)
                                .rotateBy(pose.getRotation());

    Translation2d newTranslation = pose.getTranslation().plus(offset);

    Rotation2d newHeading = pose.getRotation().plus(Rotation2d.fromDegrees(degreesOffset));

    return new Pose2d(newTranslation, newHeading);
}

public Pose2d closestTag(List<Pose2d> poseList){
    Boolean[][] listBool;
    Pose2d shortestPosey = poseList.get(0);
    double distance = 1000000000;
    for (int i = 0; i < poseList.size(); i ++){
        double tempDist = drivetrain.getState().Pose.getTranslation().getDistance(poseList.get(i).getTranslation());
        if (tempDist < distance){
            distance = tempDist;
            shortestPosey = poseList.get(i);
            indexNum = i;
        }
    }
    SmartDashboard.putNumber("Distance", distance);
    return shortestPosey;
}

    public Pose2d closestTagS(List<Pose2d> poseList, String side){
        Boolean[][] listBool;

        if (side == "left"){
            listBool = lefties;
        }else{
            listBool = righties;
        }

        Pose2d shortestPose = poseList.get(0);
        double distance = 1000000000;
        for (int i = 0; i < poseList.size(); i ++){
            boolean hasIt = false;
            double tempDist = drivetrain.getState().Pose.getTranslation().getDistance(poseList.get(i).getTranslation());
            for (int j = 0; j < listBool[i].length; j ++){
                if (listBool[i][j] == false){
                    hasIt = true;
                }
            }
            if (tempDist < distance && hasIt){
                distance = tempDist;
                shortestPose = poseList.get(i);
                indexNum = i;
            }
        }
        SmartDashboard.putNumber("Distance", distance);
        return shortestPose;
    }

    @Override
    public void periodic(){
        group1left[0] = buttonTable.getEntry("button30").getBoolean(false);
        group1left[1] = buttonTable.getEntry("button29").getBoolean(false);
        group1left[2] = buttonTable.getEntry("button28").getBoolean(false);

        group1right[0] = buttonTable.getEntry("button27").getBoolean(false);
        group1right[1] = buttonTable.getEntry("button26").getBoolean(false);
        group1right[2] = buttonTable.getEntry("button25").getBoolean(false);

        group2left[0] = buttonTable.getEntry("button36").getBoolean(false);
        group2left[1] = buttonTable.getEntry("button35").getBoolean(false);
        group2left[2] = buttonTable.getEntry("button34").getBoolean(false);

        group2right[0] = buttonTable.getEntry("button33").getBoolean(false);
        group2right[1] = buttonTable.getEntry("button32").getBoolean(false);
        group2right[2] = buttonTable.getEntry("button31").getBoolean(false);


        group3left[0] = buttonTable.getEntry("button6").getBoolean(false);
        group3left[1] = buttonTable.getEntry("button5").getBoolean(false);
        group3left[2] = buttonTable.getEntry("button4").getBoolean(false);

        group3right[0] = buttonTable.getEntry("button3").getBoolean(false);
        group3right[1] = buttonTable.getEntry("button2").getBoolean(false);
        group3right[2] = buttonTable.getEntry("button1").getBoolean(false);


        group4left[0] = buttonTable.getEntry("button12").getBoolean(false);
        group4left[1] = buttonTable.getEntry("button11").getBoolean(false);
        group4left[2] = buttonTable.getEntry("button10").getBoolean(false);

        group4right[0] = buttonTable.getEntry("button9").getBoolean(false);
        group4right[1] = buttonTable.getEntry("button8").getBoolean(false);
        group4right[2] = buttonTable.getEntry("button7").getBoolean(false);


        group5left[0] = buttonTable.getEntry("button18").getBoolean(false);
        group5left[1] = buttonTable.getEntry("button17").getBoolean(false);
        group5left[2] = buttonTable.getEntry("button16").getBoolean(false);

        group5right[0] = buttonTable.getEntry("button15").getBoolean(false);
        group5right[1] = buttonTable.getEntry("button14").getBoolean(false);
        group5right[2] = buttonTable.getEntry("button13").getBoolean(false);


        group6left[0] = buttonTable.getEntry("button24").getBoolean(false);
        group6left[1] = buttonTable.getEntry("button23").getBoolean(false);
        group6left[2] = buttonTable.getEntry("button22").getBoolean(false);

        group6right[0] = buttonTable.getEntry("button21").getBoolean(false);
        group6right[1] = buttonTable.getEntry("button20").getBoolean(false);
        group6right[2] = buttonTable.getEntry("button19").getBoolean(false);

        righties[0] = group1right;
        righties[1] = group2right;
        righties[2] = group3right;
        righties[3] = group4right;
        righties[4] = group5right;
        righties[5] = group6right;

        lefties[0] = group1left;
        lefties[1] = group2left;
        lefties[2] = group3left;
        lefties[3] = group4left;
        lefties[4] = group5left;
        lefties[5] = group6left;
        
    }


    

}
