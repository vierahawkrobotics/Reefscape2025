package frc.robot.ArmSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.ArmSubsystem.ArmConstants.HeightState;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.Components.PositionTools.PositionTools;
// import frc.robot.Drivetrain.DrivePoseBased;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import frc.robot.Components.CANdleComponent.CANdleConstants;
// import frc.robot.Components.CANdleComponent.CANdleController;
import frc.robot.Components.CANdleComponent.CANdleController;
import frc.robot.Components.CANdleComponent.CANdleConstants;
import frc.robot.Components.PositionTools.PositionTools;


public class DropCoralCommand extends Command {
    private enum DropState {
        MoveInit,
        MovePeriodic,
        Move2Init,
        Move2Periodic,
        DropInit,
        DropPeriodic,
        End
    }
    private DropState state = DropState.MoveInit;
    private Supplier<Boolean> shouldEnd;
    private Pose2d origin = new Pose2d();
    private Pose2d offset = new Pose2d(-ArmConstants.armForwardOffset,-Robot.instance.armSubsystem.isLimitSwitchPressed(),Rotation2d.fromDegrees(0));
    private Pose2d pose = PositionTools.getPoseTranslated(origin, offset);
    private HeightState height;
    private boolean isRight = false;
    public DropCoralCommand(Supplier<Boolean> shouldEnd, HeightState height, boolean isRight) {
        this.shouldEnd = shouldEnd;
        this.height = height;
        this.isRight = isRight;
        addRequirements(Robot.instance.armSubsystem);
        addRequirements(Robot.instance.drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("Drop");}
    @Override
    public void execute() {
        switch(state) {
            default:
            case MoveInit: // Set robot target position to reef
                Pose2d translatedPremove = PositionTools.getPoseTranslated(PositionTools.closestScorePoseEntry(false), pose);
                // moveCommand = new SequentialCommandGroup(new DrivePoseBased(translatedPremove,()->{return false;}), new DrivePoseBased(translateMove,()->{return false;}));
                // moveCommand.schedule();
                Robot.instance.drivetrain.setTargetPos(translatedPremove.getX(), translatedPremove.getY());
                Robot.instance.drivetrain.setTargetPosRot(translatedPremove.getRotation().getRadians());
                state = DropState.MovePeriodic;
                break;
            case MovePeriodic: // Check target
                if (Robot.instance.drivetrain.getIsPointReached()) {
                   state = DropState.Move2Init;
                }
                break;
            case Move2Init:
                Pose2d translateMove = PositionTools.getPoseTranslated(PositionTools.closestScorePose(false, 
                    Robot.instance.armSubsystem.limitSwitchOffset + (isRight ? ArmConstants.coralPipeDistance / 2: -ArmConstants.coralPipeDistance / 2)),
                    pose);
                Robot.instance.drivetrain.setTargetPos(translateMove.getX(), translateMove.getY());
                Robot.instance.drivetrain.setTargetPosRot(translateMove.getRotation().getRadians());
                Robot.instance.armSubsystem.setHeightState(height);
                state = DropState.Move2Periodic;
                break;
            case Move2Periodic:
                if (Robot.instance.drivetrain.getIsPointReached() && Robot.instance.armSubsystem.AtTargetHeight()) {
                    state = DropState.DropInit;
                }
                break;
            case DropInit: // Begin dropping
                Robot.instance.armSubsystem.setIntakeState(ArmConstants.IntakeState.Drop);
                CANdleController.setState(CANdleConstants.RobotStates.Dropping);
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
        Robot.instance.drivetrain.setVel(0,0);
        Robot.instance.drivetrain.setVelRot(0);
        CANdleController.setState(CANdleConstants.RobotStates.Idle);
    }
    @Override
    public boolean isFinished() {
        return state == DropState.End || (shouldEnd != null && shouldEnd.get());
    }
}
