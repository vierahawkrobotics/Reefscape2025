package frc.robot.Match;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Robot;
import frc.robot.Components.CANdleSubsystem.*;
import frc.robot.Drivetrain.Drive3D;

public class RobotState {
    public static XboxController controller1;
    public static XboxController controller2;
    public static void Initialize() {
        controller1 = new XboxController(0);
        controller2 = new XboxController(1);

        // Robot.instance.drivetrain.setDefaultCommand(new Drive3D(() -> {
        //     return -1*controller.getLeftY();
        // }, () -> {
        //     return controller.getLeftX();
        // }, () -> {
        //     return controller.getRightX();
        // }));

        new JoystickButton(controller1, XboxController.Button.kA.value).onTrue(new CANdleStateChangeCommand(CANdleConstants.RobotStates.Intaking));
        new JoystickButton(controller1, XboxController.Button.kB.value).onTrue(new CANdleStateChangeCommand(CANdleConstants.RobotStates.Dropping));
        new JoystickButton(controller1, XboxController.Button.kY.value).onTrue(new CANdleStateChangeCommand(CANdleConstants.RobotStates.AlgaeRemoval));
        new JoystickButton(controller1, XboxController.Button.kX.value).onTrue(new CANdleStateChangeCommand(CANdleConstants.RobotStates.Climbing));
        new JoystickButton(controller1, XboxController.Button.kLeftBumper.value).onTrue(new CANdleStateChangeCommand(CANdleConstants.RobotStates.RobotMoving));
        new JoystickButton(controller1, XboxController.Button.kRightBumper.value).onTrue(new CANdleStateChangeCommand(CANdleConstants.RobotStates.Idle));
    }
    public static void Periodic() {}
}
