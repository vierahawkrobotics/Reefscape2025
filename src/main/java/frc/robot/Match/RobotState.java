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
import frc.robot.ArmSubsystem.RemoveAlgaeRawCommand;
import frc.robot.ArmSubsystem.ElevatorMovementCommand;
import frc.robot.ArmSubsystem.ElevatorSetHeightCommand;
import frc.robot.ArmSubsystem.RemoveAlgaeCommand;
import frc.robot.ArmSubsystem.ArmConstants.HeightState;
import frc.robot.Components.PositionComponent.PositionComponent;
import frc.robot.Drivetrain.Drive2D;
import frc.robot.Drivetrain.Drive3D;
import frc.robot.Drivetrain.LimelightEnable;
import frc.robot.Drivetrain.ResetHeading;

public class RobotState {
    public static XboxController controller1;
    public static XboxController controller2;
    private static boolean toggleFixedAlign = false;
    public static void Initialize() {
        controller1 = new XboxController(0);
        controller2 = new XboxController(1);

        controller1.setRumble(RumbleType.kBothRumble, 0);
        controller2.setRumble(RumbleType.kBothRumble, 0);
        // Controller 1 (Update?)
        //   Left Joystick - Movement, Right Joystick - Rotation
        Robot.instance.drivetrain.setDefaultCommand(new Drive2D(() -> {
            return -1*controller1.getLeftY();
        }, () -> {
            return -1*controller1.getLeftX();
        }, () -> { 
            double m = Math.sqrt(controller1.getRightX()*controller1.getRightX()+controller1.getRightY()*controller1.getRightY());
            if(m < 0.4) return null;
            double k =  Math.atan2(-controller1.getRightX(),-controller1.getRightY());
            //align to hexagon
            if(controller1.getRightStickButtonPressed()) toggleFixedAlign ^= true;
            if(toggleFixedAlign) k = Math.round(k * (6 / (2 * Math.PI))) / (6 / (2 * Math.PI));
            return k;
        }, () -> {
            //use alignment rotation
            return controller1.getAButton();
        }));

        // Controller 2
        //   Container
        new JoystickButton(controller2, XboxController.Button.kA.value).onTrue(new CollectCoralCommand(()->{return !controller2.getAButton();}));
        new JoystickButton(controller2, XboxController.Button.kB.value).onTrue(new DropCoralRawCommand());
        //new JoystickButton(controller2, XboxController.Button.kY.value).onTrue(new RemoveAlgaeRawCommand(true, null));
        //new JoystickButton(controller2, XboxController.Button.kX.value).onTrue(new RemoveAlgaeRawCommand(false, null));
        new JoystickButton(controller2, XboxController.Button.kX.value).onTrue(new RemoveAlgaeCommand(ArmConstants.HeightState.AlgaeHigh, ()->{
            return controller2.getPOV() == 270;
        })); // Up - top
        new JoystickButton(controller2, XboxController.Button.kY.value).onTrue(new RemoveAlgaeCommand(ArmConstants.HeightState.AlgaeLow, ()->{
            return controller2.getPOV() == 270;
        })); // Up - top
        new JoystickButton(controller2, XboxController.Button.kLeftBumper.value).onTrue(new DropCoralCommand(null,() -> {
            return controller2.getPOV() == 270;
        }, HeightState.CoralHigh, true));
        new JoystickButton(controller2, XboxController.Button.kRightBumper.value).onTrue(new DropCoralCommand(null, () -> {
            return controller2.getPOV() == 270;
        }, HeightState.CoralHigh, false));
        new Trigger(()->{return controller2.getLeftTriggerAxis() > 0.9;}).onTrue(new DropCoralCommand(null, () -> {
            return controller2.getPOV() == 270;
        }, HeightState.CoralLow, true));
        new Trigger(()->{return controller2.getRightTriggerAxis() > 0.9;}).onTrue(new DropCoralCommand(null, () -> {
            return controller2.getPOV() == 270;
        }, HeightState.CoralLow, false));

        new JoystickButton(controller1, XboxController.Button.kLeftStick.value).onTrue(new ResetHeading());
        new JoystickButton(controller1, XboxController.Button.kStart.value).onTrue(new LimelightEnable(false));
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
    static int index = 0;
    public static void Periodic() {
        if((index++ % 100) == 0) System.out.println("robo pose: " + PositionComponent.getRobotPose());
    }
}