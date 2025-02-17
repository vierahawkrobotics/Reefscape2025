package frc.robot.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Components.CANdle.*;

enum DropState {
    PremoveInit,
    PremovePeriodic,
    MoveInit,
    MovePeriodic,
    AlignInit,
    AlignPeriodic,
    DropInit,
    DropPeriodic,
    End
}

public class DropCoralCommand extends Command {
    private DropState state = DropState.MoveInit;
    private ArmConstants.CoralDropState dropPos;
    public DropCoralCommand(ArmConstants.CoralDropState dropPos) {
        addRequirements(Robot.instance.armSubsystem);
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
                // drivetrain function move in front of place coral (include ArmConstants.armDistance)
                state = DropState.MovePeriodic;
                break;
            case PremovePeriodic: // Check target placement
                // if at target (drivetrain function)
                //    state = DropState.AlignInit;
                break;
            case MoveInit: // Set robot target position in front of coral
                break;
            case MovePeriodic: // Check target placement
                break;
            case AlignInit: // Set robot align to reef
                // drivetrain function align coral
                state = DropState.AlignPeriodic;
                break;
            case AlignPeriodic: // Check alignment
                // if aligned (drivetrain function)
                //    state = DropState.DropInit;
                break;
            case DropInit: // Begin dropping
                Robot.instance.armSubsystem.setIntakeState(ArmConstants.IntakeState.Drop);
                state = DropState.DropPeriodic;
                CANdleController.setState(CANdleConstants.RobotStates.Dropping);
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
        CANdleController.setState(CANdleConstants.RobotStates.Dropping);
    }
    @Override
    public boolean isFinished() {
        return state == DropState.End;
    }
}
