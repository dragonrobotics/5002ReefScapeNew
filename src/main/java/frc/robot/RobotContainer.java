// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.InitialSimPose;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOReal;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.Arm.ArmSetpoint;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIOReal;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.Elevator.ElevatorSetpoint;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class RobotContainer {
	// Control Interfaces
	private final CommandXboxController controller = new CommandXboxController(Constants.xboxControllerPort);
	private final Joystick buttonBoard = new Joystick(Constants.buttonBoardPort);

	// Button Board Mappings
	private final JoystickButton l2Button = new JoystickButton(buttonBoard, 1);
	private final JoystickButton l3Button = new JoystickButton(buttonBoard, 2);
	private final JoystickButton l4Button = new JoystickButton(buttonBoard, 3);
	private final JoystickButton defaultButton = new JoystickButton(buttonBoard, 5);
	private final JoystickButton intakeButton = new JoystickButton(buttonBoard, 4);
	private final JoystickButton a2Button = new JoystickButton(buttonBoard, 6);
	private final JoystickButton a3Button = new JoystickButton(buttonBoard, 7);
	private final JoystickButton climbButton = new JoystickButton(buttonBoard, 8);

	// Subsystems
	private final CommandSwerveDrivetrain swerveDrive = TunerConstants.createDrivetrain();
	private final Arm arm;
	private final Climber climber;
	private final Elevator elevator;
	private final Intake intake;
	private final Vision vision = new Vision(swerveDrive);

	// Subsystem Interfaces
	private final ArmIOReal armIOReal = new ArmIOReal();
	private final ClimberIOReal climberIOReal = new ClimberIOReal();
	private final ElevatorIOReal elevatorIOReal = new ElevatorIOReal();
	private final IntakeIOReal intakeIOReal = new IntakeIOReal();

	private final ArmIOSim armIOSim = new ArmIOSim();
	private final ClimberIOSim climberIOSim = new ClimberIOSim();
	private final ElevatorIOSim elevatorIOSim = new ElevatorIOSim();
	private final IntakeIOSim intakeIOSim = new IntakeIOSim(CommandSwerveDrivetrain.mapleSimSwerveDrivetrain);

	// Max. robot speeds
	private final double maxLinearSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
	private final double maxAngularSpeed = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

	// Robot Drive Mode
	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
		.withDeadband(maxLinearSpeed * 0.1)
		.withRotationalDeadband(maxAngularSpeed * 0.1)
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage);
	
	private final Telemetry logger = new Telemetry(maxLinearSpeed);

	public RobotContainer() {
		switch (Robot.CURRENT_DEBUG_STATE) {
			case REAL:
				arm = new Arm(armIOReal, elevatorIOReal);
				climber = new Climber(climberIOReal);
				elevator = new Elevator(elevatorIOReal);
				intake = new Intake(intakeIOReal);

				break;

			case SIM:
				arm = new Arm(armIOSim, elevatorIOSim);
				climber = new Climber(climberIOSim);
				elevator = new Elevator(elevatorIOSim);
				intake = new Intake(intakeIOSim);

				break;
		
			default:
				arm = new Arm(armIOReal, elevatorIOReal);
				climber = new Climber(climberIOReal);
				elevator = new Elevator(elevatorIOReal);
				intake = new Intake(intakeIOReal);
		}

		swerveDrive.resetPose(
			new Pose2d(
				InitialSimPose.xPosition,
				InitialSimPose.yPosition,
				new Rotation2d(
					InitialSimPose.angle
				)
			)
		);

		configureUniversalBindings();

		// Configures different bindings based on what is selected in the driver station
		if (RobotState.isTeleop()) {
			configureTeleOpBindings();
		} else if (RobotState.isTest()) {
			configureTestBindings();
		}
	}

	private void configureUniversalBindings() {
		swerveDrive.setDefaultCommand(
			swerveDrive.applyRequest(
				() -> drive
				.withVelocityX(controller.getLeftY() * maxLinearSpeed)
				.withVelocityY(controller.getLeftX() * maxLinearSpeed)
				.withRotationalRate(-controller.getRightX() * maxAngularSpeed)	
			)
		);

		swerveDrive.registerTelemetry(logger::telemeterize);

		controller.leftBumper().whileTrue(intake.intakeCoral());
		controller.rightBumper().whileTrue(intake.shootCoral());

		controller.y().whileTrue(climber.climb());
		controller.x().onTrue(climber.release());

		controller.a().onTrue(
			runOnce(
				() -> CommandScheduler.getInstance().cancelAll()
			)
		);

		controller.b().onTrue(vision.driveToTrackedAprilTag());
	
		controller.back().onTrue(
			swerveDrive.runOnce(
				() -> swerveDrive.seedFieldCentric()
			)
		);
	}

	private void configureTeleOpBindings() {
		arm.setDefaultCommand(arm.rotateToSetpoint());

		defaultButton.onTrue(
			changeState(
				ArmSetpoint.Default, 
				ElevatorSetpoint.Default
			)
		);

		intakeButton.onTrue(
			changeState(
				ArmSetpoint.Intake, 
				ElevatorSetpoint.Intake
			)
		);

		climbButton.onTrue(
			changeState(
				ArmSetpoint.Climb, 
				ElevatorSetpoint.Climb
			)
		);

		a2Button.onTrue(
			changeState(
				ArmSetpoint.A2,
				ElevatorSetpoint.A2
			)
		);

		a3Button.onTrue(
			changeState(
				ArmSetpoint.A3,
				ElevatorSetpoint.A3
			)
		);

		l2Button.onTrue(
			changeState(
				ArmSetpoint.L2,
				ElevatorSetpoint.L2
			)
		);

		l3Button.onTrue(
			changeState(
				ArmSetpoint.L3,
				ElevatorSetpoint.L3
			)
		);

		l4Button.onTrue(
			changeState(
				ArmSetpoint.L4,
				ElevatorSetpoint.L4
			)
		);
	}

	private void configureTestBindings() {
		arm.setDefaultCommand(
			arm.runArm(1)
		);

		elevator.setDefaultCommand(
			elevator.stopElevator()
		);

		controller.povLeft().whileTrue(arm.rotateCounterClockwise());
		controller.povRight().whileTrue(arm.rotateClockwise());

		controller.povUp().whileTrue(elevator.moveElevatorUp());
		controller.povDown().whileTrue(elevator.moveElevatorDown());

		controller.a().onTrue(elevator.calibrate());
		controller.b().onTrue(arm.calibrate());
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}

	public Command changeState(ArmSetpoint armSetpoint, ElevatorSetpoint elevatorSetpoint) {
		return either(getAutonomousCommand(), getAutonomousCommand(), a2Button);
	}
}
