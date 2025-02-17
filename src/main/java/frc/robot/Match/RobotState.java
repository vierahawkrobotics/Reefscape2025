package frc.robot.Match;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Robot;

import frc.robot.ArmSubsystem.CollectCoralCommand;
import frc.robot.ArmSubsystem.DropCoralCommand;
import frc.robot.ArmSubsystem.ElevatorUpDownCommand;
import frc.robot.ArmSubsystem.RemoveAlgaeCommand;
import frc.robot.Drivetrain.Drive3D;

public class RobotState {
    public static XboxController controller;
    public static void Initialize() {
        controller = new XboxController(0);
        //Defines button A which calls function null
        new JoystickButton(controller, XboxController.Button.kA.value).onTrue(null);
        //defines button B which calls function CollectCoralCommand
        new JoystickButton(controller, XboxController.Button.kB.value).onTrue(new CollectCoralCommand());
        //defines button X which calls function DropCoralCommand
        new JoystickButton(controller, XboxController.Button.kX.value).onTrue(new DropCoralCommand());
        //defines button Y which calls function RemoveAlgaeCommand
        new JoystickButton(controller, XboxController.Button.kY.value).onTrue(new RemoveAlgaeCommand());
        //defines button LB which calls function ElevatorUpDownCommand
        new JoystickButton(controller, XboxController.Button.kLeftBumper.value).onTrue(new ElevatorUpDownCommand());

        Robot.instance.drivetrain.setDefaultCommand(new Drive3D(() -> {
            return -1*controller.getLeftY();
        }, () -> {
            return controller.getLeftX();
        }, () -> {
            return controller.getRightX();
        }));
    }
    public static void Periodic() {}
}
