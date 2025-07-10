package frc.robot.Match;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Robot;
// import frc.robot.ArmSubsystem.ArmConstants;
// import frc.robot.ArmSubsystem.CollectCoralCommand;
// import frc.robot.ArmSubsystem.DropCoralCommand;
// import frc.robot.ArmSubsystem.DropCoralRawCommand;
// import frc.robot.ArmSubsystem.ElevatorMovementCommand;
// import frc.robot.ArmSubsystem.ElevatorSetHeightCommand;
// import frc.robot.ArmSubsystem.RemoveAlgaeCommand;
// import frc.robot.ArmSubsystem.ArmConstants.HeightState;
import frc.robot.Drivetrain.Drive2D;
import frc.robot.Drivetrain.Drive3D;
import frc.robot.Drivetrain.Drive3DRotate;
import frc.robot.Drivetrain.DrivetrainConstants;
import frc.robot.Drivetrain.ResetHeading;

public class RobotState {
    public static XboxController controller1;
    public static XboxController controller2;
    public static void Initialize() {
        controller1 = new XboxController(0);
        controller2 = new XboxController(1);

        // Controller 1 (Update?)
        //   Left Joystick - Movement, Right Joystick - Rotation
        Robot.instance.drivetrain.setDefaultCommand(new Drive2D(() -> {
            return -1*controller1.getLeftY()*(controller1.getRawButton(XboxController.Button.kLeftBumper.value) ? DrivetrainConstants.driveSlowingFactor : 1);
        }, () -> {
            return -1*controller1.getLeftX()*(controller1.getRawButton(XboxController.Button.kLeftBumper.value) ? DrivetrainConstants.driveSlowingFactor : 1);
        }, () -> { 
            return 1 *controller1.getRightX()*(controller1.getRawButton(XboxController.Button.kLeftBumper.value) ? DrivetrainConstants.rotSlowingFactor : 1);
        }));

        // Controller 2
        //   Container
        controller1.setRumble(RumbleType.kBothRumble, 0);
        controller2.setRumble(RumbleType.kBothRumble, 0);
        // new JoystickButton(controller2, XboxController.Button.kY.value).onTrue(new CollectCoralCommand(()->{return !controller2.getYButton();}));
        // new JoystickButton(controller2, XboxController.Button.kA.value).onTrue(new DropCoralRawCommand());
        new JoystickButton(controller1, XboxController.Button.kX.value).onTrue(new ResetHeading());

        /*new JoystickButton(controller1, XboxController.Button.kRightBumper.value).onTrue(new DropCoralCommand(() -> {
            return controller1.getBButton();
        }, HeightState.CoralHigh, true));
        new JoystickButton(controller1, XboxController.Button.kLeftBumper.value).onTrue(new DropCoralCommand(() -> {
            return controller1.getBButton();
        }, HeightState.CoralHigh, false));
        new Trigger(()->{return controller1.getLeftTriggerAxis() > 0.9;}).onTrue(new DropCoralCommand(() -> {
            return controller1.getBButton();
        }, HeightState.CoralLow, true));
        new Trigger(()->{return controller1.getRightTriggerAxis() > 0.9;}).onTrue(new DropCoralCommand(() -> {
            return controller1.getBButton();
        }, HeightState.CoralLow, false));*/
        // //   Elevator
        // new Trigger(()->{return controller2.getPOV() == 0;}).onTrue(new ElevatorSetHeightCommand(ArmConstants.HeightState.CoralHigh)); // Up - top
        // new Trigger(()->{return controller2.getPOV() == 90;}).onTrue(new ElevatorSetHeightCommand(ArmConstants.HeightState.CoralLow)); // Right - mid
        // new Trigger(()->{return controller2.getPOV() == 270;}).onTrue(new ElevatorMovementCommand(0,true,false))
        //                                                       .onFalse(new RemoveAlgaeCommand(()->{return true;})); // Algae Cycle
        // new Trigger(()->{return controller2.getPOV() == 180;}).onTrue(new ElevatorSetHeightCommand(ArmConstants.HeightState.Ground)); // Down - bo'om
        //   Algae
        //new JoystickButton(controller2, XboxController.Button.kX.value).onTrue(new RemoveAlgaeCommand(()->{return false;}));
        //   Climber
        //climber command (X)

        //Jansen's alternative alignment 
        //Todo figure out what buttons he wants
        double lTheta = 0;
        Drive3DRotate alternativeDrive = new Drive3DRotate(
            ()->{return controller1.getLeftY();},
            ()->{return controller1.getLeftX();},
            ()->{
                double y = -1*controller1.getRightY();
                double x = controller1.getRightX();
                if(Math.sqrt(y*y+x*x) < 0.1) return null;
                
                return (Math.round(Math.atan2(y,x) * 3 / Math.PI) * Math.PI / 3) % (Math.PI*2);
            }
        );
        new JoystickButton(controller1, XboxController.Button.kRightBumper.value).onTrue(
            alternativeDrive
        ).onFalse(
            new InstantCommand(alternativeDrive::cancel)
        );
    }
    public static void Periodic() {
    }
}