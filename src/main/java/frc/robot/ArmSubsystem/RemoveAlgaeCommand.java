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
    int y;
    public RemoveAlgaeCommand(int y) {
        addRequirements(Robot.instance.armSubsystem);
        addRequirements(Robot.instance.drivetrainSubsystem);
        this.y = y;
    }

    @Override
    public void initialize() {
        switch(state) {
            case SetupInit:
                Robot.instance.armSubsystem.setPosition(y);
                //set robot position
                Robot.instance.drivetrainSubsystem.setPosition(TriggerEffect.getAlgeaPose(y));
                break;
            case SetupPeriodic:
                if(drivetrain.isAtTargetPose() && arm.isAtTargetHeight())
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
                //rotat4e eject wheels and move arm up
                //robot move back a foot
                

        }
    }
    @Override
    public void execute() {}
    @Override
    public void end(boolean interrupted) {}
    @Override
    public boolean isFinished() {
        return false;
    }
} 