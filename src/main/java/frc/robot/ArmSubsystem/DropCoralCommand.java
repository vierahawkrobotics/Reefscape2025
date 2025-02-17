package frc.robot.ArmSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Components.PositionTools.PositionTools;
import frc.robot.Drivetrain.DrivePoseBased;

enum DropState {
    PremoveInit,
    PremovePeriodic,
    DropInit,
    DropPeriodic,
    End
}

public class DropCoralCommand extends Command {
    private DropState state = DropState.MoveInit;
    private ArmConstants.CoralDropState dropPos;
    private SequentialCommandGroup moveCommand;
    public DropCoralCommand(ArmConstants.CoralDropState dropPos) {
        addRequirements(Robot.instance.armSubsystem);
        addRequirements(Robot.instance.drivetrain);
        this.dropPos = dropPos;
    }

    @Override
    public void initialize() {}
    @Override
    public void execute() {
        switch(state) {
            default:
            case PremoveInit: // Set robot target position to where won't hit wall
                Robot.instance.armSubsystem.setHeightState(dropPos.getHeight());
                // Replace DrivePoseBased params with closestScorePoseEntry 
                Pose2d entryPose = PositionTools.closestScorePoseEntry(isScheduled());
                moveCommand = new SequentialCommandGroup(new DrivePoseBased(0, 0, 0, null),
                new DrivePoseBased(0, 0, 0, null));
                moveCommand.schedule();
                state = DropState.PremoveInit;
                break;
            case PremovePeriodic: // Check target placement
                // if done (drivetrain)
                state = DropState.DropInit;
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
