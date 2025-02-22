// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
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

    public SendableChooser<Boolean> calibrationMode = new SendableChooser<>();
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        calibrationMode.setDefaultOption("Competition", false);
        calibrationMode.addOption("Calibration", true);

        autoChooser = AutoBuilder.buildAutoChooser("Test Auto");

        SmartDashboard.putData("Auto Mode", autoChooser);
        SmartDashboard.putData("Mode", calibrationMode);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
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

        if(calibrationMode.getSelected().booleanValue() == true){
            
          }
          else{
            joystick.povDown().onTrue(elevatorDown());
      
            joystick.povRight().onTrue(elevatorMid());
      
            joystick.povUp().whileTrue(elevatorUp());

            joystick.b().onTrue(stopElevator());


            //joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

            joystick.a().onTrue(calibrateElevator());
            //joystick.leftTrigger().onTrue(intake());
            }

        drivetrain.registerTelemetry(logger::telemeterize);
    }

     //Moves elevator to different positions, will be revised
  public Command elevatorDown(){
    return runOnce(()-> {elevator.moveToPosition(0.0);}, elevator);
  }

  public Command elevatorMid(){
    return runOnce(()-> {elevator.moveToPosition(14.0);}, elevator);
  }

  public Command elevatorUp(){
    return run(()-> {elevator.setMotor(0.75);}, elevator);
  }

  public Command stopElevator(){
    return run(()-> {elevator.stopMotor();}, elevator);
  }

  //Calibrates the Elevator conversion factor from the bottom. MAKE SURE IT STARTS AT THE BOTTOM
  public Command calibrateElevator(){
    return sequence(
      runOnce(() -> {elevator.zeroEncoder();}, elevator),
      runOnce(() -> {elevator.setMotor(0.35);}, elevator),
      waitUntil(elevator.atTop()),
      runOnce(() -> {elevator.stopMotor();}, elevator),
      runOnce(() -> {elevator.calibrate();}, elevator),
      elevator.moveToPosition(0.0)
    );
  }

  public Command ArmSide(){
    return run(()->{arm.moveToPosition(60.0);}, arm).until(arm.atGoal());
  }


  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
