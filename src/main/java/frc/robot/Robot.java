// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Components.*;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Drivetrain.DrivetrainConstants;
import frc.robot.Match.*;
import frc.robot.subsystemExample.ExampleSubsystem;
import frc.robot.Testing.*;

public class Robot extends TimedRobot {
  public static Robot instance;
  public ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
  public XboxController controller = new XboxController(DrivetrainConstants.usbPortController);
  public Drivetrain drivetrain = new Drivetrain();
  @Override
  public void robotInit() {
    GUI.initialize();
    ComponentManager.Initialize();
    instance = this;

    RobotState.Initialize();
  }

  @Override
  public void robotPeriodic() {
    GUI.periodic();
    ComponentManager.Periodic();
    CommandScheduler.getInstance().run();
    RobotState.Periodic();
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
    
  }

  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void teleopExit() {
    
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
