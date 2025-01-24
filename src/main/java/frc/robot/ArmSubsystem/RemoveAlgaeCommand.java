package frc.robot.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

enum RemoveAlgaeState {
    SetupInit,
    SetupPeriodic,
    ExtendInit,
    ExtendPeriodic,
    EjectInit,
    EjectPeriodic,
    End
}
enum heightState{
    High,
    Low
}
public class RemoveAlgaeCommand extends Command {
    private RemoveAlgaeState state = RemoveAlgaeState.SetupInit;
    private heightState hState;
    public RemoveAlgaeCommand(heightState targetHeight) {
        addRequirements(Robot.instance.armSubsystem);
        addRequirements(Robot.instance.drivetrainSubsystem);
        this.hState = targetHeight;
    }

    @Override
    public void initialize() {

    }
    @Override
    public void execute() {
        switch(state) {
            case SetupInit:
                Robot.instance.armSubsystem.setPosition(this.hState);
                //set robot position
                Robot.instance.drivetrainSubsystem.setPosition(TriggerEffect.getAlgeaPose(this.hState));
                state = RemoveAlgaeState.SetupPeriodic;
                break;
            case SetupPeriodic:
                if(drivetrain.isAtTargetPose() && Robot.instance.armSubsystem.atTargetHeight())
                    state = RemoveAlgaeState.ExtendInit;
                break;
            case ExtendInit:
                //move robot forward
                state = RemoveAlgaeState.ExtendPeriodic;
                break;
            case ExtendPeriodic:
                if(drivetrain.isAtTargetPose())
                    state = RemoveAlgaeState.EjectInit;
                break;
            case EjectInit:
                Robot.instance.armSubsystem.ejectAlgae();
                state = RemoveAlgaeState.EjectPeriodic;
                break;
            case EjectPeriodic:
                // (move robot back a foot) Robot.instance.drivetrainSubsystem.
                //arm.ejectAlgae()
                //rotate eject wheels and move arm up
                //robot move back a foot
                if(IsFinished()){
                    state = RemoveAlgaeState.End;
                }
                break;
        }            
    }
    @Override
    public void end(boolean interrupted) {}
    @Override
    public boolean IsFinished() {
        if(state == RemoveAlgaeState.End){
            return true;
        }

    }
}