package frc.robot.ArmSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class DropCoralCommand extends Command {
    public static double startTime;
    private DropState state = DropState.MoveInit;
    public DropCoralCommand() {
        addRequirements(Robot.instance.exampleSubsystem);
    }

    @Override
    public void initialize() {}
    @Override
    public void execute() {
        switch(state) {
            case MoveInit: // Set robot position
                // drivetrain function move in front of place coral (include ArmConstants.armDistance)
                state = DropState.MovePeriodic;
                break;
            case MovePeriodic: // Check in front of coral placement
                // if at target (drivetrain function)
                //    state = DropState.AlignInit;
                break;
            case AlignInit: // Align placement to reef
                // drivetrain function align place coral
                state = DropState.AlignPeriodic;
                break;
            case AlignPeriodic: // Check alignment
                // if aligned (drivetrain function)
                //    state = DropState.DropInit;
                break;
            case DropInit: // Begin dropping
                ArmSubsystem.setIntakeState(ArmSubsystem.IntakeState.Drop);
                startTime = Timer.getFPGATimestamp();
                state = DropState.DropPeriodic;
                break;
            case DropPeriodic: // Check done dropping
                if(isFinished()){
                    state = DropState.End;
                }
                break;
        }
    }
    @Override
    public void end(boolean interrupted) {}
    @Override
    public boolean isFinished() {
        if (ArmSubsystem.getIntakeState() == ArmSubsystem.IntakeState.Rest) {
            return true;
        } else {
            return false;
        }
    }

    enum DropState {
        MoveInit,
        MovePeriodic,
        AlignInit,
        AlignPeriodic,
        DropInit,
        DropPeriodic,
        End
    }
}
