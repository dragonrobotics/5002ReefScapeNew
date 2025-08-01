// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.security.KeyStore.PrivateKeyEntry;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision;
@Logged
public class Robot extends TimedRobot {
private Command m_autonomousCommand;
NetworkTable table;


private final RobotContainer m_robotContainer;
private final boolean kUseLimelight = false;
SendableChooser<Boolean> toggleChooser = new SendableChooser<>();
public Robot() {
  Epilogue.bind(this);
  m_robotContainer = new RobotContainer();

  
}

@Override
public void robotInit(){
  PortForwarder.add(5800,"photonvision", 5800);
  table = NetworkTableInstance.getDefault().getTable("VirtualButtonBoard");
}

@Override
public void robotPeriodic() {
  boolean intakeval = table.getEntry("button1").getBoolean(false);

    SmartDashboard.putBoolean("VALUE", intakeval);
  CommandScheduler.getInstance().run(); 
}

@Override
public void disabledInit() {}

@Override
public void disabledPeriodic() {}

@Override
public void disabledExit() {}

@Override
public void autonomousInit() {
  m_autonomousCommand = m_robotContainer.getAutonomousCommand();
  m_robotContainer.configureBindings();

  if (m_autonomousCommand != null) {
    m_autonomousCommand.schedule();
  }
}

@Override
public void autonomousPeriodic() {
  
}

@Override
public void autonomousExit() {}

@Override
public void teleopInit() {
  if (m_autonomousCommand != null) {
    m_autonomousCommand.cancel();
  }

  m_robotContainer.configureBindings();   
}

@Override
public void teleopPeriodic() {
}

@Override
public void teleopExit() {}

@Override
public void testInit() {
  CommandScheduler.getInstance().cancelAll();
}

@Override
public void testPeriodic() {}

@Override
public void testExit() {}

@Override
public void simulationPeriodic() {}
}
