package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.none;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.swerve.jni.SwerveJNI.DriveState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Paths extends SubsystemBase {
    private CommandSwerveDrivetrain drivetrain;
    public Command currentCommand;
    private List<Pose2d> completedPosesRight = new ArrayList<>();
    private List<Pose2d> completedPosesLeft = new ArrayList<>();
    private List<Pose2d> completedPoses = new ArrayList<>();

    private PathConstraints contraints; 
    public Paths(CommandSwerveDrivetrain driveTrain){
        drivetrain = driveTrain;
        contraints = new PathConstraints(3.0, 4.0,
                                        Units.degreesToRadians(540), Units.degreesToRadians(720));
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
        Pose2d shortestPose = poseList.get(0);
        double distance = 1000000000;
        for (int i = 0; i < poseList.size(); i ++){
            double tempDist = drivetrain.getState().Pose.getTranslation().getDistance(poseList.get(i).getTranslation());
            if (tempDist < distance){
                distance = tempDist;
                shortestPose = poseList.get(i);
            }
        }
        SmartDashboard.putNumber("Distance", distance);
        return shortestPose;
    }

    

}
