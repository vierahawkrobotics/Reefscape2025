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
    double height;
    public RemoveAlgaeCommand() {
        addRequirements(Robot.instance.armSubsystem);
        addRequirements(Robot.instance.drivetrain);
        height = Robot.instance.armSubsystem.getTargetHeight();
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        switch(state) {
            default:
            case SetupInit://initialize arm and robot position
                //set robot position, 
                //setTargetPos(double posX, double posY) in meters
                //setTargetRot(double posR) in radians for posR
                ///Robot.instance.Drivetrain.setTargetPos(TriggerEffect.getAlgeaPose(this.hState));
                /// -----------------OR-----------------
                /// Another option is to make a new DrivePoseBased command and set it on the command
                /// scheduler which will allow you to use the isFinished method to check if the robot
                ///  is at the target pose
                state = RemoveAlgaeState.SetupPeriodic;
                break;
            case SetupPeriodic://check if at target pose
                ///if(DrivetrainPoseBased.isFinished() && Robot.instance.armSubsystem.atTargetHeight())
                    state = RemoveAlgaeState.ExtendInit;
                break;
            case ExtendInit://extend arm
                //move robot forward
                state = RemoveAlgaeState.ExtendPeriodic;
                break;
            case ExtendPeriodic://check if at target pose
                /// if(DrivetrainPoseBased.isFinished())
                    state = RemoveAlgaeState.EjectInit;
                break;
            case EjectInit://eject algae
                if (height == ArmConstants.HeightState.CoralLow.getHeight() || height == ArmConstants.HeightState.Collect.getHeight() || height == ArmConstants.HeightState.Ground.getHeight()) {
                    Robot.instance.armSubsystem.setHeightState(ArmConstants.HeightState.AlgaeLow);
                } else {
                    Robot.instance.armSubsystem.setHeightState(ArmConstants.HeightState.AlgaeHigh);
                }
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