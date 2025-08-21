package frc.robot.RobotStates;

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

        Drivetrain.getInstance().setDefaultCommand(new JoystickControl(() -> {
            return -1* controller.getLeftY();
        }, () -> {
            return  -1 * controller.getLeftX();
        }, () -> {
            return -1* controller.getRightX();
        }
        ));

        new JoystickButton(controller, ControllerBindings.RESETPOSE).onTrue(new InstantCommand(PositionComponent::zeroPos));
        new JoystickButton(controller, ControllerBindings.TOGGLEHOLD).toggleOnTrue(new HoldPosition());
    }
    public static void Periodic() {}
}
