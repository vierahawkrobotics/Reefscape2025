package frc.robot.Match;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Robot;
import frc.robot.ArmSubsystem.ArmConstants;
import frc.robot.ArmSubsystem.CollectCoralCommand;
import frc.robot.ArmSubsystem.DropCoralCommand;
import frc.robot.ArmSubsystem.DropCoralRawCommand;
import frc.robot.ArmSubsystem.ElevatorMovementCommand;
import frc.robot.ArmSubsystem.ElevatorSetHeightCommand;
import frc.robot.ArmSubsystem.RemoveAlgaeCommand;
import frc.robot.ArmSubsystem.ArmConstants.HeightState;
import frc.robot.Drivetrain.Drive2D;
import frc.robot.Drivetrain.Drive3D;
import frc.robot.Drivetrain.ResetHeading;

public class RobotState {
    public static XboxController controller1;
    public static XboxController controller2;
    public static void Initialize() {
        controller1 = new XboxController(0);
        controller2 = new XboxController(1);

        // Controller 1 (Update?)
        //   Left Joystick - Movement, Right Joystick - Rotation
        Robot.instance.drivetrain.setDefaultCommand(new Drive3D(() -> {
            return -1*controller1.getLeftY();
        }, () -> {
            return -1*controller1.getLeftX();
        }, () -> { 
            return -1 *controller1.getRightX();
        }));

        // Controller 2
        //   Container
        controller1.setRumble(RumbleType.kBothRumble, 0);
        controller2.setRumble(RumbleType.kBothRumble, 0);
        new JoystickButton(controller2, XboxController.Button.kY.value).onTrue(new CollectCoralCommand(()->{return !controller2.getYButton();}));
        new JoystickButton(controller2, XboxController.Button.kA.value).onTrue(new DropCoralRawCommand());
        new JoystickButton(controller2, XboxController.Button.kX.value).onTrue(new ResetHeading());

        new JoystickButton(controller2, XboxController.Button.kRightBumper.value).onTrue(new DropCoralCommand(() -> {
            return controller2.getBButton();
        }, HeightState.CoralHigh, true));
        new JoystickButton(controller2, XboxController.Button.kLeftBumper.value).onTrue(new DropCoralCommand(() -> {
            return controller2.getBButton();
        }, HeightState.CoralHigh, false));
        new Trigger(()->{return controller2.getLeftTriggerAxis() > 0.9;}).onTrue(new DropCoralCommand(() -> {
            return controller2.getBButton();
        }, HeightState.CoralLow, true));
        new Trigger(()->{return controller2.getRightTriggerAxis() > 0.9;}).onTrue(new DropCoralCommand(() -> {
            return controller2.getBButton();
        }, HeightState.CoralLow, false));
        //   Elevator
        new Trigger(()->{return controller2.getPOV() == 0;}).onTrue(new ElevatorSetHeightCommand(ArmConstants.HeightState.CoralHigh)); // Up - top
        new Trigger(()->{return controller2.getPOV() == 90;}).onTrue(new ElevatorSetHeightCommand(ArmConstants.HeightState.CoralLow)); // Right - mid
        // new Trigger(()->{return controller2.getPOV() == 270;}).onTrue(new ElevatorMovementCommand(0,true,false))
        //                                                       .onFalse(new RemoveAlgaeCommand(()->{return true;})); // Algae Cycle
        new Trigger(()->{return controller2.getPOV() == 180;}).onTrue(new ElevatorSetHeightCommand(ArmConstants.HeightState.Ground)); // Down - bo'om
        //   Algae
        // new JoystickButton(controller2, XboxController.Button.kX.value).onTrue(new RemoveAlgaeCommand(()->{return false;}));
        //   Climber
        //climber command (X)
    }
    public static void Periodic() {
    }
}