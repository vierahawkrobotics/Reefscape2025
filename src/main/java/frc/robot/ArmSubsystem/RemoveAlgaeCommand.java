package frc.robot.ArmSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.ArmSubsystem.ArmConstants.HeightState;
import frc.robot.Components.PositionTools.PositionTools;
import frc.robot.Components.CANdleComponent.CANdleConstants;
import frc.robot.Components.CANdleComponent.CANdleController;

enum RemoveAlgaeState {
    MoveInit,
    MovePeriodic,
    Move2Init,
    Move2Periodic,
    EjectInit,
    EjectPeriodic,
    End
}

public class RemoveAlgaeCommand extends Command {
    private RemoveAlgaeState state = RemoveAlgaeState.MoveInit;
    private Supplier<Boolean> interrupted;
    ArmConstants.HeightState height;
    private Pose2d pose;
    public RemoveAlgaeCommand(ArmConstants.HeightState height, Supplier<Boolean> interrupted) {
        addRequirements(Robot.instance.armSubsystem);
        addRequirements(Robot.instance.drivetrain);
        this.height = height;
        this.interrupted = interrupted;
    }

    @Override
    public void initialize() {
        state = RemoveAlgaeState.MoveInit;
        Pose2d origin = new Pose2d(0, 0, Rotation2d.kZero);
        Pose2d offset = new Pose2d(ArmConstants.algeaArmOffsetX,ArmConstants.algeaArmOffsetY,Rotation2d.fromDegrees(0));
        pose = PositionTools.getPoseTranslated(origin, offset);
        System.out.println("algae");
        System.out.println("pose: " + pose);
        CANdleController.setColor(CANdleConstants.RobotStates.Dropping);
    }
    
    @Override
    public void execute() {
        switch(state) {
            default:
            case MoveInit: // Set robot target position to reef
                Pose2d translatedPremove = PositionTools.getPoseTranslated(PositionTools.closestScorePoseEntry(true), Pose2d.kZero);
                // moveCommand = new SequentialCommandGroup(new DrivePoseBased(translatedPremove,()->{return false;}), new DrivePoseBased(translateMove,()->{return false;}));
                // moveCommand.schedule();
                Robot.instance.drivetrain.setTargetPos(translatedPremove.getX(), translatedPremove.getY());
                Robot.instance.drivetrain.setTargetPosRot(translatedPremove.getRotation().getRadians());
                state = RemoveAlgaeState.MovePeriodic;
                System.out.println("premove: " + translatedPremove);
                break;
            case MovePeriodic: // Check target
                if (Robot.instance.drivetrain.getIsPointReached(0.1)) {
                   state = RemoveAlgaeState.Move2Init;
                }
                break;
            case Move2Init:
                double d = -Robot.instance.armSubsystem.getPrevLimitSwitchOffset();
                Pose2d translateMove = PositionTools.getPoseTranslated(PositionTools.closestScorePose(true, 0),pose);
                Robot.instance.drivetrain.setTargetPos(translateMove.getX(), translateMove.getY());
                Robot.instance.drivetrain.setTargetPosRot(translateMove.getRotation().getRadians());
                Robot.instance.armSubsystem.setHeightState(height);
                Robot.instance.armSubsystem.setAlgaeMotorSpeed(ArmConstants.AlgaeMotorState.ActiveTemp);
                state = RemoveAlgaeState.Move2Periodic;
                System.out.println("move: " + translateMove);
                System.out.println("move d: " + d);
                break;
            case Move2Periodic:
                if (Robot.instance.drivetrain.getIsPointReached(0.04) && Robot.instance.drivetrain.getIsRotationReached() &&
                 Robot.instance.drivetrain.checkIsRobotStopped() && Robot.instance.armSubsystem.AtTargetHeight()) {
                    state = RemoveAlgaeState.EjectInit;
                }
                break;
            case EjectInit: // Begin algae ejection
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
        System.out.println("end algae");
        CANdleController.setColor(CANdleConstants.RobotStates.Idle);
    }
    @Override
    public boolean isFinished() {
        return state == RemoveAlgaeState.End || (interrupted != null && interrupted.get());
    }
}
