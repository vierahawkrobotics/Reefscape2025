package frc.robot.Match;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Robot;

import frc.robot.Drivetrain.Drive3D;

public class RobotState {
    public static XboxController controller1;
    public static XboxController controller2;
    public static void Initialize() {
        controller1 = new XboxController(0);
        controller2 = new XboxController(1);

        // Controller 1
        //  Left Joystick - Movement, Right Joystick - Rotation
        Robot.instance.drivetrain.setDefaultCommand(new Drive3D(() -> {
            return controller1.getLeftY();
        }, () -> {
            return controller1.getLeftX();
        }, () -> {
            return controller1.getRightX();
        }));

        // Controller 2
        //  Container

        //  Elevator

        //  Climber
    }
    public static void Periodic() {}
}