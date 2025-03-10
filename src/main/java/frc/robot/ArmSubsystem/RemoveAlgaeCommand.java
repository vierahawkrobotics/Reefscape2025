package frc.robot.ArmSubsystem;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

enum RemoveAlgaeState {
    SetupInit,
    SetupPeriodic,
    EjectInit,
    EjectPeriodic,
    End
}

public class RemoveAlgaeCommand extends Command {
    private RemoveAlgaeState state = RemoveAlgaeState.SetupInit;
    double height;
    private Supplier<Boolean> interrupted;
    public RemoveAlgaeCommand(Supplier<Boolean> interrupted) {
        addRequirements(Robot.instance.armSubsystem);
        height = Robot.instance.armSubsystem.getTargetHeight();
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        switch(state) {
            default:
            case SetupInit:// Begin arm extension
                if (height == ArmConstants.HeightState.CoralLow.getHeight() || height == ArmConstants.HeightState.Collect.getHeight() || height == ArmConstants.HeightState.Ground.getHeight()) {
                    Robot.instance.armSubsystem.setHeightState(ArmConstants.HeightState.AlgaeLow);
                } else if (height == ArmConstants.HeightState.CoralHigh.getHeight()){
                    Robot.instance.armSubsystem.setHeightState(ArmConstants.HeightState.AlgaeHigh);
                }
                state = RemoveAlgaeState.SetupPeriodic;
                break;
            case SetupPeriodic:// Check at target pose and height
                if (Robot.instance.armSubsystem.AtTargetHeight()) { 
                    state = RemoveAlgaeState.EjectInit;
                }
                break;
            case EjectInit: // Begin algae ejection
                Robot.instance.armSubsystem.setAlgaeMotorSpeed(ArmConstants.AlgaeMotorState.ActiveTemp);
                state = RemoveAlgaeState.EjectPeriodic;
                break;
            case EjectPeriodic:// Check at target height
                if (Robot.instance.armSubsystem.getAlgaeState() == ArmConstants.AlgaeMotorState.Inactive) {
                    state = RemoveAlgaeState.End;
                }
                break;
        }            
    }
    @Override
    public void end(boolean interrupted) {
        Robot.instance.armSubsystem.setHeightState(ArmConstants.HeightState.Ground);
        Robot.instance.armSubsystem.setAlgaeMotorSpeed(ArmConstants.AlgaeMotorState.Inactive);
    }
    @Override
    public boolean isFinished() {
        return state == RemoveAlgaeState.End || interrupted.get();
    }
}
