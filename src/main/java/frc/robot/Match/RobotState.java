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
    public static XboxController controller1;
    public static XboxController controller2;
    public static void Initialize() {
        controller1 = new XboxController(0);
        controller2 = new XboxController(1);
        // Defines button B which calls function CollectCoralCommand
        new JoystickButton(controller1, XboxController.Button.kB.value).onTrue(new CollectCoralCommand());
        // Defines button X which calls function DropCoralCommand
        new JoystickButton(controller1, XboxController.Button.kX.value).onTrue(new DropCoralCommand());
        // Defines button left bumper which calls function RemoveAlgaeCommand
        new JoystickButton(controller1, XboxController.Button.kLeftBumper.value).onTrue(new RemoveAlgaeCommand());
        // Defines button Y which moves elevator up
        new JoystickButton(controller1, XboxController.Button.kY.value).onTrue(new ElevatorUpDownCommand(true,false));
        // Defines button A which moves elevator up
        new JoystickButton(controller1, XboxController.Button.kA.value).onTrue(new ElevatorUpDownCommand(false,true));

        Robot.instance.drivetrain.setDefaultCommand(new Drive3D(() -> {
            return -1*controller1.getLeftY();
        }, () -> {
            return controller1.getLeftX();
        }, () -> {
            return controller1.getRightX();
        }));
    }
    public static void Periodic() {}
}