package frc.robot.ArmSubsystem;
import javax.lang.model.util.ElementScanner14;

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
    Low;
    double getHeight() {
        switch(this){
            case High:
                return ArmConstants.high;
            case Low:
                return ArmConstants.low;
            default:
                return ArmConstants.ground;
        }
    }
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
    public void initialize() {}
    
    @Override
    public void execute() {
        switch(state) {
            case SetupInit://initialize arm and robot position
                ArmSubsystem.SetTargetHeight(hState.getHeight());
                //set robot position
                ///Robot.instance.drivetrainSubsystem.setPosition(TriggerEffect.getAlgeaPose(this.hState));
                state = RemoveAlgaeState.SetupPeriodic;
                break;
            case SetupPeriodic://check if at target pose
                ///if(drivetrain.isAtTargetPose() && Robot.instance.armSubsystem.atTargetHeight())
                    state = RemoveAlgaeState.ExtendInit;
                break;
            case ExtendInit://extend arm
                //move robot forward
                state = RemoveAlgaeState.ExtendPeriodic;
                break;
            case ExtendPeriodic://check if at target pose
                ///if(drivetrain.isAtTargetPose())
                    state = RemoveAlgaeState.EjectInit;
                break;
            case EjectInit://eject algae
                Robot.instance.armSubsystem.ejectAlgae();
                state = RemoveAlgaeState.EjectPeriodic;
                break;
            case EjectPeriodic://check if at target height
                // (move robot back a foot) Robot.instance.drivetrainSubsystem.
                //arm.ejectAlgae()
                //rotate eject wheels and move arm up
                //robot move back a foot
                if(isFinished()){
                    state = RemoveAlgaeState.End;
                }
                break;
        }            
    }
    @Override
    public void end(boolean interrupted) {}
    @Override
    public boolean isFinished() {
        if(ArmSubsystem.AtTargetHeight() == true){
            return true;
        }
        else{
            return false;
        }

    }
}