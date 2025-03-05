package frc.robot.Match;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Robot;
import frc.robot.ArmSubsystem.CollectCoralCommand;
import frc.robot.ArmSubsystem.DropCoralCommand;
import frc.robot.ArmSubsystem.ElevatorMovementCommand;
import frc.robot.ArmSubsystem.RemoveAlgaeCommand;

public class RobotState {
    public static XboxController controller1;
    public static XboxController controller2;
    public static void Initialize() {
        controller1 = new XboxController(0);
        controller2 = new XboxController(1);

        // Controller 1
        //   Left Joystick - Movement, Right Joystick - Rotation
        // Robot.instance.drivetrain.setDefaultCommand(new Drive3D(() -> {
        //     return controller1.getLeftY();
        // }, () -> {
        //     return controller1.getLeftX();
        // }, () -> { 
        //     return controller1.getRightX();
        // }));

        // Controller 2
        //   Container
        controller2.setRumble(RumbleType.kBothRumble, 1);
        new JoystickButton(controller2, XboxController.Button.kY.value).onTrue(new CollectCoralCommand(()->{return false;}))
                                                                       .onTrue(new InstantCommand(()->{controller2.setRumble(RumbleType.kBothRumble, 1);}))
                                                                       .onFalse(new CollectCoralCommand(()->{return true;}))
                                                                       .onFalse(new InstantCommand(()->{controller2.setRumble(RumbleType.kBothRumble, 1);}));
        new JoystickButton(controller2, XboxController.Button.kA.value).onTrue(new DropCoralCommand());
        new JoystickButton(controller2, XboxController.Button.kB.value).onTrue(new RunCommand(()->{Robot.instance.armSubsystem.SetTargetHeight(1);}, Robot.instance.armSubsystem));
        //   Elevator
        new Trigger(()->{return controller2.getPOV() == 0;}).onTrue(new ElevatorMovementCommand(2,false,false)); // Up
        new Trigger(()->{return controller2.getPOV() == 180;}).onTrue(new ElevatorMovementCommand(1,false,false)); // Down
                                                              // On False Turn off Algae Motor
        new Trigger(()->{return controller2.getPOV() == 270;}).onTrue(new ElevatorMovementCommand(0,true,false)); // Algae Cycle
        new Trigger(()->{return controller2.getPOV() == 90;}).onTrue(new ElevatorMovementCommand(0,false,true)); // Reset
        //   Algae
        new JoystickButton(controller2, XboxController.Button.kX.value).onTrue(new RemoveAlgaeCommand());
        //   Climber
        //climber command call (button X)
    }
    public static void Periodic() {
    }
}