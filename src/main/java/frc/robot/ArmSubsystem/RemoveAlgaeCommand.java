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

public class RemoveAlgaeCommand extends Command {
    private RemoveAlgaeState state = RemoveAlgaeState.SetupInit;
    ArmConstants.AlgaeDropState target;
    public RemoveAlgaeCommand(ArmConstants.AlgaeDropState target) {
        addRequirements(Robot.instance.armSubsystem);
        addRequirements(Robot.instance.drivetrain);
        this.target = target;
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        switch(state) {
            default:
            case SetupInit://initialize arm and robot position
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
                Robot.instance.armSubsystem.setHeightState(target.getHeight());
                state = RemoveAlgaeState.EjectPeriodic;
                break;
            case EjectPeriodic://check if at target height
                // (move robot back a foot) Robot.instance.drivetrainSubsystem.
                //arm.ejectAlgae()
                //rotate eject wheels and move arm up
                //robot move back a foot
                Robot.instance.armSubsystem.setAlgaeMotorSpeed(ArmConstants.AlgaeMotorState.Active);
                if(Robot.instance.armSubsystem.AtTargetHeight()){
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
        return state == RemoveAlgaeState.End;
    }
}