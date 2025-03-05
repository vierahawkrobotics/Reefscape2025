package frc.robot.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import frc.robot.Robot;

public class CollectCoralCommand extends Command {
    private Supplier<Boolean> interrupted;
    public CollectCoralCommand(Supplier<Boolean> interrupted) {
        addRequirements(Robot.instance.armSubsystem);
        this.interrupted = interrupted;
    }

    @Override
    public void initialize() {
        Robot.instance.armSubsystem.setHeightState(ArmConstants.HeightState.Collect);
        Robot.instance.armSubsystem.setIntakeState(ArmConstants.IntakeState.Collect);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        Robot.instance.armSubsystem.setHeightState(ArmConstants.HeightState.Ground);
        Robot.instance.armSubsystem.setIntakeState(ArmConstants.IntakeState.Rest);
    }

    @Override
    public boolean isFinished() {
        return Robot.instance.armSubsystem.getIntakeState() == ArmConstants.IntakeState.Rest || interrupted.get();
    }
}
