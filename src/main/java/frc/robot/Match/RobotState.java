package frc.robot.Match;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Robot;
import frc.robot.Components.PositionComponent;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Drivetrain.HoldPosition;
import frc.robot.Drivetrain.JoystickControl;

public class RobotState {
    public static XboxController controller;
    public static void Initialize() {
        controller = new XboxController(0);
        // new JoystickButton(controller, XboxController.Button.kA.value).onTrue(null);

        // Drivetrain.getInstance().setDefaultCommand(new JoystickControl(() -> {
        //     return -1* controller.getLeftY();
        // }, () -> {
        //     return  -1 * controller.getLeftX();
        // }, () -> {
        //     return -1* controller.getRightX();
        // }
        // ));
        Drivetrain.getInstance().setDefaultCommand(new HoldPosition());

        new JoystickButton(controller, XboxController.Button.kA.value).onTrue(new InstantCommand(PositionComponent::zeroPos));
    }
    public static void Periodic() {}
}
