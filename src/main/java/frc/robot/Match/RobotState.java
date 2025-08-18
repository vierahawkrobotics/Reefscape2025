package frc.robot.Match;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Robot;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Drivetrain.JoystickControl;

public class RobotState {
    public static XboxController controller;
    public static void Initialize() {
        controller = new XboxController(0);
        // new JoystickButton(controller, XboxController.Button.kA.value).onTrue(null);

        Drivetrain.getInstance().setDefaultCommand(new JoystickControl(() -> {
            return -1* controller.getLeftY();
        }, () -> {
            return  -1 * controller.getLeftX();
        }, () -> {
            return controller.getRightX();
        }
        ));

    }
    public static void Periodic() {}
}
