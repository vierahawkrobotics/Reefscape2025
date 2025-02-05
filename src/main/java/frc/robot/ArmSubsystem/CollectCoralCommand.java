package frc.robot.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class CollectCoralCommand extends Command {
    public CollectCoralCommand() {
        addRequirements(Robot.instance.exampleSubsystem);
    }

    @Override
    public void initialize() {
        ArmSubsystem.SetTargetHeight(ArmConstants.collectHeight);
        ArmSubsystem.setIntakeState(ArmSubsystem.IntakeState.Collect);
    }

    @Override
    public void execute() {}

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
}
