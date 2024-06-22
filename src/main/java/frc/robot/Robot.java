// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystemExample.ExampleSubsystem;
import frc.robot.Testing.*;
import frc.robot.Match.*;

public class Robot extends TimedRobot {
  public static ExampleSubsystem exampleSubsystem = new ExampleSubsystem();

  @Override
  public void robotInit() {
    GUI.initialize();
  }

  @Override
  public void robotPeriodic() {
    GUI.periodic();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  private Command autoCommand;
  @Override
  public void autonomousInit() {
    autoCommand = AutonomousState.getAutoCommand();
    if (autoCommand != null) autoCommand.schedule();
    AutonomousState.initialize();
  }

  @Override
  public void autonomousPeriodic() {
    AutonomousState.periodic();
  }

  @Override
  public void autonomousExit() {
    if (autoCommand != null) autoCommand.cancel();
    AutonomousState.exit();
  }

  @Override
  public void teleopInit() {
    TeleopState.initialize();
  }

  @Override
  public void teleopPeriodic() {
    TeleopState.periodic();
  }

  @Override
  public void teleopExit() {
    TeleopState.exit();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    TestingGUI.initialize();
    TestingState.initialize();
  }

  @Override
  public void testPeriodic() {
    TestingGUI.periodic();
    TestingState.periodic();
  }

  @Override
  public void testExit() {
    TestingState.exit();
  }
}
