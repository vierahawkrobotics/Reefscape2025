package frc.robot.Match;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Robot;
import frc.robot.Drivetrain.Drive3D;

public class RobotState {
    public static XboxController controller;
    public static void Initialize() {
        controller = new XboxController(0);
        //new JoystickButton(controller, XboxController.Button.kA.value).onTrue(null);

        Robot.instance.drivetrain.setDefaultCommand(new Drive3D(() -> {
            return controller.getLeftY();
        }, () -> {
            return controller.getLeftX();
        }, () -> {
            return controller.getRightX();
        }));
    }
    public static void Periodic() {}
}
