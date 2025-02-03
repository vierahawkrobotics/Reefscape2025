package frc.robot.ArmSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class DropCoralCommand extends Command {
    public static double startTime;
    private DropState state;
    public DropCoralCommand() {
        addRequirements(Robot.instance.exampleSubsystem);
    }

    @Override
    public void initialize() {}
    @Override
    public void execute() {
        switch(state) {
            case SetupInit:
                Robot.instance.armSubsystem.setPosition(this.hState);
                //set robot position
                Robot.instance.drivetrainSubsystem.setPosition(TriggerEffect.getAlgeaPose(this.hState));
                state = DropState.SetupPeriodic;
                break;
            case SetupPeriodic:
                if(drivetrain.isAtTargetPose() && Robot.instance.armSubsystem.atTargetHeight())
                    state = DropState.ExtendInit;
                break;
            case ExtendInit:
                //move robot forward
                state = DropState.ExtendPeriodic;
                break;
            case ExtendPeriodic:
                if(drivetrain.isAtTargetPose())
                    state = DropState.EjectInit;
                break;
            case DropInit:
                Robot.instance.armSubsystem.ejectAlgae();
                state = DropState.EjectPeriodic;
                break;
            case DropPeriodic:
                // (move robot back a foot) Robot.instance.drivetrainSubsystem.
                //arm.ejectAlgae()
                //rotate eject wheels and move arm up
                //robot move back a foot
                if(IsFinished()){
                    state = DropState.End;
                }
                break;
        }
        startTime = Timer.getFPGATimestamp();
        ArmSubsystem.setIntakeState(ArmSubsystem.IntakeState.Drop);
    }
    @Override
    public void end(boolean interrupted) {}
    @Override
    public boolean isFinished() {
        if (ArmSubsystem.getIntakeState() == ArmSubsystem.IntakeState.Rest) {
            return true;
        } else {
            return false;
        }
    }

    enum DropState {
        SetupInit,
        SetupPeriodic,
        ExtendInit,
        ExtendPeriodic,
        DropInit,
        DropPeriodic,
        End
    }
}
