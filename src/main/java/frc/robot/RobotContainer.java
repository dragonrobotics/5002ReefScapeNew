// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;

import static edu.wpi.first.wpilibj2.command.Commands.none;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Elevator elevator = new Elevator();
    public final Arm arm = new Arm();

    public SendableChooser<Boolean> calibrationMode = new SendableChooser<Boolean>();
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        
      autoChooser = AutoBuilder.buildAutoChooser("Test Auto");
      configureBindings();

        
    }

    private void configureBindings() {
        calibrationMode.setDefaultOption("Competition", false);
        calibrationMode.addOption("Calibration", true);

        SmartDashboard.putData("Auto Mode", autoChooser);
        SmartDashboard.putData("Mode", calibrationMode);

        if(calibrationMode.getSelected() == null){
          System.out.println("NO MODE");
        }
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        joystick.x().onTrue(ArmSide());
        joystick.rightTrigger().onTrue(calibrateArm());
        joystick.leftTrigger().onTrue(ArmSide());

        if(calibrationMode.getSelected()){
          System.out.println("CALIBRATING");
          
          }
        else{
          joystick.a().onTrue(calibrateElevator());
          joystick.y().onTrue(elevatorTop());
    
          joystick.povRight().onTrue(elevatorMid());

          joystick.povDown().whileTrue(elevatorDown());
    
          joystick.povUp().whileTrue(elevatorUp());

          joystick.b().onTrue(stopElevator());

          //joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

          
          //joystick.leftTrigger().onTrue(intake());

          
          elevator.setDefaultCommand(elevator.runElevator());

            }

        drivetrain.registerTelemetry(logger::telemeterize);
    }

     //Moves elevator to different positions, will be revised
  

  public Command elevatorTop(){
    return runOnce(()-> {elevator.moveToPosition(27.0);}, elevator);
  }

  public Command elevatorMid(){
    return runOnce(()-> {elevator.moveToPosition(14.0);}, elevator);
  }

  public Command elevatorBottom(){
    return runOnce(()-> {elevator.moveToPosition(1.0);}, elevator);
  }

  public Command elevatorUp(){
    return run(()-> {elevator.setMotor(0.3);}, elevator);
  }

  public Command elevatorDown(){
    return run(()-> {elevator.setMotor(-0.3);}, elevator);
  }

  public Command stopElevator(){
    return run(()-> {elevator.stopMotor();}, elevator);
  }

  //Calibrates the Elevator conversion factor from the bottom. MAKE SURE IT STARTS AT THE BOTTOM
  public Command calibrateElevator(){
    return sequence(
      runOnce(() -> {elevator.zeroEncoder();}, elevator),
      run(() -> {elevator.setMotor(0.35);}, elevator).until
      (elevator.atTop()),
      runOnce(()->{System.out.println(elevator.getAmps());}),
      runOnce(() -> {elevator.stopMotor();}, elevator),
      runOnce(() -> {elevator.calibrate();}, elevator)
    );
  }

  public Command calibrateArm(){
    return sequence(
      runOnce(() -> {arm.zero();}, arm)
    );
  }

  public Command ArmSide(){
    return run(()->{arm.moveToPosition(60.0);}, arm);
  }
  public Command stopArm(){
    return runOnce(()->{arm.stop();});
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
