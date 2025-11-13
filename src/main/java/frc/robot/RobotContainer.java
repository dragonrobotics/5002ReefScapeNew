// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.json.simple.JSONObject;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.nio.file.Files;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.autoRotate;
import frc.robot.subsystems.vision;
import frc.robot.subsystems.Paths;

import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.none;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.jar.Attributes.Name;

@Logged
public class RobotContainer {

    public double xPos = 0.0;
    public double yPos = 0.0;
    
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.RobotCentric roboDrive = new SwerveRequest.RobotCentric(); // Use open-loop control for drive motors
    //private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  //private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // public static CommandXboxController joystick = new CommandXboxController(0);
    public static CommandXboxController joystick = new CommandXboxController(0);
    private final Joystick coJoystick = new Joystick(1);
    private final Joystick aviator = new Joystick(3);

    private final JoystickButton fireButton = new JoystickButton(aviator, 1);
    private final JoystickButton fbackn = new JoystickButton(aviator, 5);


    private final JoystickButton l2Button = new JoystickButton(aviator, 16);
    private final JoystickButton l3Button = new JoystickButton(aviator, 9);
    private final JoystickButton l4Button = new JoystickButton(aviator, 10);
    private final JoystickButton defaultButton = new JoystickButton(aviator, 15);
    private final JoystickButton intakeButton = new JoystickButton(coJoystick,24);
    private final JoystickButton a2Button = new JoystickButton(coJoystick, 6);
    private final JoystickButton a3Button = new JoystickButton(coJoystick, 7);
    private final JoystickButton climbButton = new JoystickButton(coJoystick, 8);

    public final CommandSwerveDrivetrain drivetrain; 
    public final Paths path;
    //public final autoAlign align; 
    public final autoRotate align;
    public final static Elevator elevator = new Elevator();
    public final static Arm arm = new Arm();
    public final static Intake intake = new Intake();
    public final Climber climber = new Climber();
    public final vision Vision;
    public SendableChooser<Boolean> mode = new SendableChooser<Boolean>();
    private final SendableChooser<Command> autoChooser;
    public Boolean isRightPose;
    public List<Pose2d> poseList = new ArrayList<>();
    public List<Pose2d> intakePoseList = new ArrayList<>();
    public List<Pose2d> completedPoses = new ArrayList<>();
    public Command currentCommand;

    public RobotContainer() {
      isRightPose = true;
      // NetworkTable table = NetworkTableInstance.getDefault().getTable("VirtualButtonBoard");
      // boolean intakeval = table.getEntry("intake").getBoolean(false);

      // SmartDashboard.putBoolean("VALUE", intakeval);

      SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
      //DataLogManager.start();      

      mode.setDefaultOption("Competition", false);
      mode.addOption("Calibrate", true);

      SmartDashboard.putData("Mode", mode);

      currentCommand = none();

      if(mode.getSelected() == null){
        System.out.println("NO MODE");
      
      }

      //REGISTER AUTO COMMANDS
      NamedCommands.registerCommand("intake", intakeOnce());
      NamedCommands.registerCommand("shoot", shootOnce());
      NamedCommands.registerCommand("l4", l4State());
      NamedCommands.registerCommand("l3", l3State());
      NamedCommands.registerCommand("l2", l2State());
      NamedCommands.registerCommand("defaultState", defaultState());
      NamedCommands.registerCommand("collectState", collectState());
      NamedCommands.registerCommand("climbState", climbState());
      NamedCommands.registerCommand("runElevator", elevator.runElevator());
      NamedCommands.registerCommand("runArm", arm.runArm());
      NamedCommands.registerCommand("stop", stopShoot());
      
      drivetrain = TunerConstants.createDrivetrain();
      autoChooser = AutoBuilder.buildAutoChooser("Middle L4 Intake");
      SmartDashboard.putData("Auto Mode", autoChooser);
      
      Vision = new vision(drivetrain);
      align = new autoRotate(drivetrain, Vision, joystick);
      path = new Paths(drivetrain);
      if (DriverStation.getAlliance().isPresent()){
        DriverStation.Alliance alliance = DriverStation.getAlliance().get();

        if  (alliance == DriverStation.Alliance.Red){

          for (int i = 0; i < Constants.redAutoPoses.length; i ++){
            poseList.add(Constants.redAutoPoses[i]);

          }

          for (int i = 0; i < Constants.redIntakePoses.length; i ++){
            intakePoseList.add(Constants.redIntakePoses[i]);          
          }

        }else if (alliance == DriverStation.Alliance.Blue){

          for (int i = 0; i < Constants.blueAutoPoses.length; i ++){
            poseList.add(Constants.blueAutoPoses[i]);
          }

          for (int i = 0; i < Constants.blueIntakePoses.length; i ++){
            intakePoseList.add(Constants.blueIntakePoses[i]);          
          }

        }
      }

    }

    public void configureBindings() {
      if(mode.getSelected() == null){
        System.out.println("NO MODE");
      }
      // Note that X is defined as forward according to WPILib convention,
      // and Y is defined as to the left according to WPILib convention.
      // drivetrain.setDefaultCommand(
      //     // Drivetrain will execute this command periodically
      //     drivetrain.applyRequest(() ->
      //         drive.withVelocityX(aviator.getY() * MaxSpeed/8.0) // Drive forward with negative Y (forward)
      //             .withVelocityY(aviator.getX() * MaxSpeed/8.0) // Drive left with negative X (left)
      //             .withRotationalRate(-aviator.getZ() * MaxAngularRate*0.15) // Drive counterclockwise with negative X (left)
      //     )
      // );

      drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() ->
            drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-joystick.getRightX()* MaxAngularRate) // Drive counterclockwise with negative X (left)
        )
    );

      //UNIVERSAL BINDS

      //Reset Field Orientation
      joystick.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

      joystick.rightBumper().whileTrue(
          drivetrain.applyRequest(()->
              drive.withVelocityY(0.25)
        ).finallyDo(()->
      drivetrain.applyRequest(() ->
               drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                   .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                   .withRotationalRate(-joystick.getRightX() * MaxAngularRate)))); // Drive counterclockwise with negative X (left));
      
      //Auto Allign auwto April Tag
      // joystick.leftBumper().onTrue(runOnce(()->{Command pathCommand = path.pathTo(Paths.transformOffset(path.closestTagS(poseList, "left"), 40, 0.0, 180.0));;
      //                                           currentCommand = pathCommand; 
      //                                           pathCommand.schedule();}));

      // joystick.rightBumper().onTrue(runOnce(()->{Command pathCommand = path.pathTO(Vision.getPoseTracked());;
      // currentCommand = pathCommand; 
      // pathCommand.schedule();}));

      // joystick.rightBumper().whileTrue(run(()->{
      //       Pose2d poser = Vision.fixResult(6.0);
      //       Command align = Vision.alignToPose(poser,6);
      //       if (align != null){
      //         currentCommand = align;
      //         currentCommand.schedule();
      //       };
    
      // }).finallyDo(()->
      // drivetrain.applyRequest(() ->
      //         drive.withVelocityX(-oystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
      //             .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
      //             .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
      //     )));


    

      // joystick.b().onTrue(runOnce(()->{Command pathCommand = Paths.pathTo(Paths.transformOffset(path.closestTag(intakePoseList), 36,10, 180.0));
      //                                           currentCommand = sequence(pathCommand);
      //                                           pathCommand.schedule();}));
  

      
                                                
      // joystick.a().onTrue(runOnce(()->{currentCommand.cancel();}));  

      //Use Shooter
      joystick.rightTrigger().whileTrue(shoot());
      fireButton.whileTrue(shoot());
      joystick.leftTrigger().whileTrue(intake());
      // joystick.square().whileTrue(climb());
      // joystick.circle().whileTrue(Unclimb());

      // joystick.povUp().whileTrue(run(()->{
  //       Command align = Vision.mover(0.25);
  //         currentCommand = align;
  //         currentCommand.schedule();

  // }).finallyDo(()->
  // drivetrain.applyRequest(() ->
  //         drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
  //             .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
  //             .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
  //     )));




      //CALIBRATION MODE BINDS
      if(mode.getSelected() == true){
        System.out.println("CALIBRATING");
        elevator.setDefaultCommand(elevator.maintanElevator());
        arm.setDefaultCommand(run(()->{arm.runMotor(0);}, arm));

      //Zero Arm and Elevator
        // joystick.a().onTrue(calibrateElevator());
        // joystick.b().onTrue(calibrateArm());

        //Move Elevator
        joystick.povUp().whileTrue(elevatorUp());
        joystick.povDown().whileTrue(elevatorDown());

        //Move Arm
        joystick.povLeft().whileTrue(armCounterClockwise());
        joystick.povRight().whileTrue(armClockwise());
        }
      //COMPETITION MODE BINDS
      else{
        System.out.println("COMPETITION");

        //Default State
        defaultButton.onTrue(defaultState());

        //Collect State
        intakeButton.onTrue(collectState());

        //Score Coralt
       
        l2Button.onTrue(l2State());
        l3Button.onTrue(l3State());
        l4Button.onTrue(l4State());
        climbButton.onTrue(climbState());

        //remove algea
        a2Button.onTrue(algeaBot());
        // joystick.rightBumper().onTrue(algeaTop());
      
        elevator.setDefaultCommand(elevator.runElevator());
        arm.setDefaultCommand(arm.runArm());
        }

      drivetrain.registerTelemetry(logger::telemeterize);

    }

     //Moves elevator to different positions, will be revised
  
  public BooleanSupplier armElevatorAtGoal(){
    return ()->arm.atGoal().getAsBoolean() && elevator.atGoal().getAsBoolean();
  }

  public Command changeState(double elevatorPosition, double armPosition){
    return
      either(
        sequence(
          runOnce(()-> System.out.println("Ran")),
          runOnce(()->{arm.setPosition(0);}),
          waitUntil(()->Math.abs(arm.getMeasurement()) < 30),
          runOnce(()->{elevator.moveToPosition(elevatorPosition);}),
          waitUntil(()->elevator.getMeasurement() > elevatorPosition - 5 && elevator.getMeasurement() < elevatorPosition + 5),
          runOnce(()->{arm.setPosition(armPosition);})),
        sequence(
          runOnce(()-> System.out.println("Ran")),
          runOnce(()->{elevator.moveToPosition(elevatorPosition);}),
          waitUntil(()->elevator.getMeasurement() > elevatorPosition - 5 && elevator.getMeasurement() < elevatorPosition + 5),
          runOnce(()->{arm.setPosition(armPosition);})),
      ()->arm.getMeasurement() < 0 || (Math.abs(arm.getMeasurement()) > 90 && elevatorPosition < elevator.getMeasurement())).until(armElevatorAtGoal()).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  //ELEVATOR COMMANDS
  public Command elevatorUp(){
    return run(()-> {elevator.setMotor(0.3);}, elevator).finallyDo(()->elevator.stopMotor());
  }

  public Command elevatorDown(){
    return run(()-> {elevator.setMotor(-0.3);}, elevator).finallyDo(()->elevator.stopMotor());
  }

  public Command stopElevator(){
    return run(()-> {elevator.stopMotor();}, elevator);
  }

  //Calibrates the Elevator conversion factor from the bottom. MAKE SURE IT STARTS AT THE BOTTOM
  public Command calibrateElevator(){
    return runOnce(()->elevator.zeroEncoder());
  }

  //ARM COMMANDS
  public Command calibrateArm(){
     return runOnce(() -> {arm.zero();}, arm);
  }


  public Command armCounterClockwise(){
    return run(()->arm.runMotor(1.0)).finallyDo(()->arm.stop());
  }

  public Command armClockwise(){
    return run(()->arm.runMotor(-1.0)).finallyDo(()->arm.stop());
  }

  public Command stopArm(){
    return runOnce(()->{arm.stop();});
  }

  //INTAKE COMMANDS
  public Command intake(){
    return run(()->{intake.runIntake(4);}).finallyDo(()->intake.runIntake(0.5));
  }

  public Command setIntake(double speed){
    return run(()->{intake.runIntake(speed);});
  }

  public Command shoot(){
    return run(()->{intake.runIntake(-4);},  intake).finallyDo(()->intake.runIntake(0.5));
  }
  public Command intakeOnce(){
    return sequence(runOnce(()->{intake.runIntake(-8);}, intake), waitSeconds(0.5), runOnce(()->{intake.stopIntake();}, intake));
  }

  public Command shootOnce(){
    return sequence(runOnce(()->{intake.runIntake(8);}, intake), waitSeconds(2), runOnce(()->{intake.stopIntake();}, intake));
  }
  public Command stopShoot(){
    return runOnce(()->{intake.runIntake(0.0);}, intake);
  }
  //Climber Commands
  public Command climb(){
    return run(()->{climber.runClimber(6.0);}).finallyDo(()->climber.stop());
  }

  public Command Unclimb(){
    return run(()->{climber.runClimber(-6.0);}).finallyDo(()->climber.stop());
  }

  //SEQUENCE COMMANDS
  public Command defaultState(){
    return changeState(1.0, 0.0);
  }

  public Command collectState(){
    return changeState(19.87, 147.43);
  }

  public Command l4State(){
    return changeState(27.95, -41.8); // 27.95 inches
  }

  public Command l3State(){
    return changeState(15.25, -36); // 15.25 inches
  }

  public Command l2State(){
    return changeState(7.31, -36); // 7.31 inches
  }

  public Command climbState(){
    return changeState(0.5, -60);
  }

  public Command algeaTop(){
    return changeState(12.25, -45);
  }

  public Command algeaBot(){
    return changeState(4.5, -45);
  }

  // public Command Align(boolean left){
  //   return run(()->{
  //     align.moveToState(left);}).
  //           until(()->((drivetrain.getState().Pose.getX() >= align.returnGoalPose2d(left).getX() -0.05 && drivetrain.getState().Pose.getX() <= align.returnGoalPose2d(left).getX() + 0.05) &&
  //                     (drivetrain.getState().Pose.getY() >= align.returnGoalPose2d(left).getY() - 0.05 && drivetrain.getState().Pose.getY() <= align.returnGoalPose2d(left).getY() + 0.05))).
  //           finallyDo(()->{
  //             drive
  //             .withVelocityX(-joystick.getLeftY() * MaxSpeed)
  //             .withVelocityY(-joystick.getLeftX() * MaxSpeed)
  //             .withRotationalRate(-joystick.getRightX() * MaxAngularRate);
  //           });
  // }



  
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  }

