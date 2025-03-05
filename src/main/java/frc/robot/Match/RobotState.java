package frc.robot.Match;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Robot;

public class RobotState {
    public static XboxController controller;
    public static void Initialize() {
        controller = new XboxController(0);
        new JoystickButton(controller, XboxController.Button.kA.value).onTrue(null);
        new JoystickButton(controller, XboxController.Button.kB.value).onTrue(new );

    }
    public static void Periodic() {}
}
