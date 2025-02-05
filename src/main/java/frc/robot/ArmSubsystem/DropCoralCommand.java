package frc.robot.ArmSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class DropCoralCommand extends Command {
    public static double startTime;
    private DropState state;
    public DropCoralCommand() {
        addRequirements(Robot.instance.exampleSubsystem);
    }

    @Override
    public void initialize() {}
    @Override
    public void execute() {
        switch(state) {
            case SetupInit: // Set robot position
                // drivetrain function move in front of place coral (include ArmConstants.armDistance)
                state = DropState.SetupPeriodic;
                break;
            case SetupPeriodic:
                // if at target (drivetrain function)
                //    state = DropState.AlignInit;
                break;
            case AlignInit:
                // drivetrain function align center to place coral
                state = DropState.AlignPeriodic;
                break;
            case AlignPeriodic:
                // if aligned (drivetrain function)
                //    state = DropState.DropInit;
                break;
            case DropInit:
                ArmSubsystem.setIntakeState(ArmSubsystem.IntakeState.Drop);
                startTime = Timer.getFPGATimestamp();
                state = DropState.DropPeriodic;
                break;
            case DropPeriodic:
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
        SetupInit,
        SetupPeriodic,
        AlignInit,
        AlignPeriodic,
        DropInit,
        DropPeriodic,
        End
    }
}
