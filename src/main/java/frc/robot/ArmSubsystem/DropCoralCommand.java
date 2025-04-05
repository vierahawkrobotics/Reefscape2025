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
    private Supplier<Boolean> shouldNotShoot;
    private Pose2d pose;
    private HeightState height;
    private ArmConstants.ReefOffset offset;
    public DropCoralCommand(Supplier<Boolean> shouldNotShoot, Supplier<Boolean> shouldEnd, HeightState height, ArmConstants.ReefOffset offset) {
        this.shouldEnd = shouldEnd;
        this.shouldNotShoot = shouldNotShoot;
        this.height = height;
        this.offset = offset;

        addRequirements(Robot.instance.armSubsystem);
        addRequirements(Robot.instance.drivetrain);
    }

    @Override
    public void initialize() {
        state = DropState.MoveInit;
        System.out.println("Drop");
    
        Pose2d origin = new Pose2d(0, 0, Rotation2d.kZero);
        Pose2d offset = new Pose2d(height == HeightState.CoralHigh ? -ArmConstants.armForwardOffsetHigh : -ArmConstants.armForwardOffsetLow,0,Rotation2d.fromDegrees(0));
        pose = PositionTools.getPoseTranslated(origin, offset);
        System.out.println("pose: " + pose);
        System.out.println("limit switch" +  Robot.instance.armSubsystem.getPrevLimitSwitchOffset());
        CANdleController.setColor(CANdleConstants.RobotStates.Dropping);
    }
    @Override
    public void execute() {
        switch(state) {
            default:
            case MoveInit: // Set robot target position to reef
                Pose2d translatedPremove = PositionTools.getPoseTranslated(PositionTools.closestScorePoseEntry(false), Pose2d.kZero);
                // moveCommand = new SequentialCommandGroup(new DrivePoseBased(translatedPremove,()->{return false;}), new DrivePoseBased(translateMove,()->{return false;}));
                // moveCommand.schedule();
                Robot.instance.drivetrain.setTargetPos(translatedPremove.getX(), translatedPremove.getY());
                Robot.instance.drivetrain.setTargetPosRot(translatedPremove.getRotation().getRadians());
                state = DropState.MovePeriodic;
                System.out.println("premove: " + translatedPremove);
                break;
            case MovePeriodic: // Check target
                if (Robot.instance.drivetrain.getIsPointReached(0.04)) {
                   state = DropState.Move2Init;
                }
                break;
            case Move2Init:
                double d = -Robot.instance.armSubsystem.getPrevLimitSwitchOffset();
                Pose2d translateMove = PositionTools.getPoseTranslated(PositionTools.closestScorePose(false, 
                    d + offset.getOffset()),
                    pose);
                Robot.instance.drivetrain.setTargetPos(translateMove.getX(), translateMove.getY());
                Robot.instance.drivetrain.setTargetPosRot(translateMove.getRotation().getRadians());
                Robot.instance.armSubsystem.setHeightState(height);
                state = DropState.Move2Periodic;
                System.out.println("move: " + translateMove);
                System.out.println("move d: " + d);
                break;
            case Move2Periodic:
                if (Robot.instance.drivetrain.getIsPointReached(0.005) && Robot.instance.drivetrain.getIsRotationReached(0.07) &&
                    Robot.instance.drivetrain.checkIsRobotStopped() && Robot.instance.armSubsystem.AtTargetHeight()) {
                    if(shouldNotShoot != null && shouldNotShoot.get()) {
                        state = DropState.End;
                    }
                    else {
                        state = DropState.DropInit;
                    }
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
        if(!interrupted && shouldNotShoot != null && shouldNotShoot.get()) {
            
        }
        else {
            Robot.instance.armSubsystem.setHeightState(ArmConstants.HeightState.Ground);
        }

        Robot.instance.armSubsystem.setIntakeState(ArmConstants.IntakeState.Rest);
        Robot.instance.drivetrain.setVelocity(0,0);
        Robot.instance.drivetrain.setVelocityRot(0);
        CANdleController.setState(CANdleConstants.RobotStates.Idle);
        System.out.println("end coral");
    }
    @Override
    public boolean isFinished() {
        return state == DropState.End || (shouldEnd != null && shouldEnd.get());
    }
}
