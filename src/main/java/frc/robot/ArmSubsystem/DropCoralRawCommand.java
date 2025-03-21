package frc.robot.ArmSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
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



public class DropCoralRawCommand extends Command {
    private enum DropState {
        MoveInit,
        MovePeriodic,
        Move2Init,
        Move2Periodic,
        DropInit,
        DropPeriodic,
        End
    }
    private DropState state = DropState.DropInit;
    // private SequentialCommandGroup moveCommand;
    // private Pose2d origin = new Pose2d();
    // private Pose2d offset = new Pose2d(ArmConstants.armForwardOffset,Robot.instance.armSubsystem.isLimitSwitchPressed(),Rotation2d.fromDegrees(0));
    // private Pose2d pose = PositionTools.getPoseTranslated(origin, offset);

    public DropCoralRawCommand() {
        addRequirements(Robot.instance.armSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Drop");
        Robot.instance.armSubsystem.setIntakeState(ArmConstants.IntakeState.Drop);
        CANdleController.setState(CANdleConstants.RobotStates.Dropping);
    }
    @Override
    public void execute() {}
    @Override
    public void end(boolean interrupted) {
        Robot.instance.armSubsystem.setHeightState(ArmConstants.HeightState.Ground);
        Robot.instance.armSubsystem.setIntakeState(ArmConstants.IntakeState.Rest);
        CANdleController.setState(CANdleConstants.RobotStates.Idle);
    }
    @Override
    public boolean isFinished() {
        return Robot.instance.armSubsystem.getIntakeState() == ArmConstants.IntakeState.Rest;
    }
}
