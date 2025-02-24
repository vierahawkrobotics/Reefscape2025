package frc.robot.ArmSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Components.PositionTools.PositionTools;
import frc.robot.Drivetrain.DrivePoseBased;

enum DropState {
    MoveInit,
    MovePeriodic,
    DropInit,
    DropPeriodic,
    End
}

public class DropCoralCommand extends Command {
    private DropState state = DropState.MoveInit;
    private SequentialCommandGroup moveCommand;

    public DropCoralCommand() {
        addRequirements(Robot.instance.armSubsystem);
        addRequirements(Robot.instance.drivetrain);
    }

    @Override
    public void initialize() {}
    @Override
    public void execute() {
        switch(state) {
            default:
            case MoveInit: // Set robot target position to reef
                moveCommand = new SequentialCommandGroup(new DrivePoseBased(PositionTools.closestScorePoseEntry(false),()->{return false;}), new DrivePoseBased(PositionTools.closestScorePose(false,Robot.instance.armSubsystem.limitSwitchOffset),()->{return false;}));
                moveCommand.schedule();
                state = DropState.MovePeriodic;
                break;
            case MovePeriodic: // Check target
                if (Robot.instance.drivetrain.DrivePoseBased.isFinished()) {
                    state = DropState.DropInit;
                }
                break;
            case DropInit: // Begin dropping
                Robot.instance.armSubsystem.setIntakeState(ArmConstants.IntakeState.Drop);
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
    }
    @Override
    public boolean isFinished() {
        return state == DropState.End;
    }
}
