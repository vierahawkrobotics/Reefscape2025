package frc.robot.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.Components.PositionTools.PositionTools;
// import frc.robot.Drivetrain.DrivePoseBased;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import frc.robot.Components.CANdleComponent.CANdleConstants;
// import frc.robot.Components.CANdleComponent.CANdleController;

enum DropState {
    MoveInit,
    MovePeriodic,
    DropInit,
    DropPeriodic,
    End
}

/**
 * @author Andrew S
 */
public class DropCoralCommand extends Command {
    private DropState state = DropState.MoveInit;
    // private SequentialCommandGroup moveCommand;
    // private Pose2d origin = new Pose2d();
    // private Pose2d offset = new Pose2d(ArmConstants.armForwardOffset,Robot.instance.armSubsystem.isLimitSwitchPressed(),Rotation2d.fromDegrees(0));
    // private Pose2d pose = PositionTools.getPoseTranslated(origin, offset);

    public DropCoralCommand() {
        addRequirements(Robot.instance.armSubsystem);
        addRequirements(Robot.instance.drivetrain);
    }

    @Override
    public void initialize() {}
    @Override
    public void execute() {
        System.out.println("Drop");
        switch(state) {
            default:
            case MoveInit: // Set robot target position to reef
                // Pose2d translatedPremove = PositionTools.getPoseTranslated(PositionTools.closestScorePoseEntry(false), pose);
                // Pose2d translateMove = PositionTools.getPoseTranslated(PositionTools.closestScorePose(false, Robot.instance.armSubsystem.limitSwitchOffset), pose);
                // moveCommand = new SequentialCommandGroup(new DrivePoseBased(translatedPremove,()->{return false;}), new DrivePoseBased(translateMove,()->{return false;}));
                // moveCommand.schedule();
                state = DropState.MovePeriodic;
                break;
            case MovePeriodic: // Check target
                // if (Robot.instance.drivetrain.getIsPointReached()) {
                   state = DropState.DropInit;
                // }
                break;
            case DropInit: // Begin dropping
                Robot.instance.armSubsystem.setIntakeState(ArmConstants.IntakeState.Drop);
                // CANdleController.setState(CANdleConstants.RobotStates.Dropping);
                state = DropState.DropPeriodic;
                break;
            case DropPeriodic: // Check done dropping
                if(Robot.instance.armSubsystem.getIntakeState() == ArmConstants.IntakeState.Rest){
                    state = DropState.End;
                }
                break;
        }
    }
    @Override
    public void end(boolean interrupted) {
        Robot.instance.armSubsystem.setHeightState(ArmConstants.HeightState.Ground);
        Robot.instance.armSubsystem.setIntakeState(ArmConstants.IntakeState.Rest);
        // CANdleController.setState(CANdleConstants.RobotStates.Idle);
    }
    @Override
    public boolean isFinished() {
        return state == DropState.End;
    }
}
