package frc.robot.Match;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Robot;
import frc.robot.ArmSubsystem.CollectCoralCommand;
import frc.robot.ArmSubsystem.DropCoralCommand;
import frc.robot.ArmSubsystem.ElevatorUpDownCommand;
import frc.robot.ArmSubsystem.RemoveAlgaeCommand;
import frc.robot.Climber.climbersubmarine;
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
        new JoystickButton(controller2, XboxController.Button.kY.value).onTrue(new CollectCoralCommand());
        new JoystickButton(controller2, XboxController.Button.kA.value).onTrue(new DropCoralCommand());
        //  Elevator
        new JoystickButton(controller2, XboxController.Button.kLeftBumper.value).onTrue(new ElevatorUpDownCommand(true));
        new JoystickButton(controller2, XboxController.Button.kRightBumper.value).onTrue(new ElevatorUpDownCommand(false));
        //  Algae
        new JoystickButton(controller2, XboxController.Button.kX.value).onTrue(new RemoveAlgaeCommand());
        //  Climber
        //Insert climber command call (button X)
    }
    public static void Periodic() {}
}