// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.ArmSubsystem.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Components.*;
import frc.robot.Components.PositionComponent.PositionComponent;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Match.*;
import frc.robot.Testing.*;

public class Robot extends TimedRobot {
  ///use a to climb
  public static Robot instance;
  public ArmSubsystem armSubsystem = new ArmSubsystem();
  public Drivetrain drivetrain = new Drivetrain();
  @Override
  public void robotInit() {
    GUI.initialize();
    ComponentManager.Initialize();
    //TODO: check
    // PositionComponent.zeroPos();
    instance = this;

    RobotState.Initialize();

    
    //Reset Pose
    // new JoystickButton(RobotState.controller1, 8).onTrue(new InstantCommand(PositionComponent::zeroPos));
  }

  @Override
  public void robotPeriodic() {
    GUI.periodic();
    ComponentManager.Periodic();
    CommandScheduler.getInstance().run();
    RobotState.Periodic();
  }

  @Override
  public void disabledInit() {
    DisabledState.Initialize();
  }

  @Override
  public void disabledPeriodic() {
    DisabledState.Periodic();
  }

  @Override
  public void disabledExit() {
    DisabledState.Exit();
  }

  private Command autoCommand;
  @Override
  public void autonomousInit() {
    AutonomousState.initialize();
    autoCommand = AutonomousState.getAutonomousCommand();
    if (autoCommand != null) autoCommand.schedule();
    
  }

  @Override
  public void autonomousPeriodic() {
    //AutonomousState.periodic();
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
    CommandScheduler.getInstance().schedule();
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
