package frc.robot.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ElevatorUpDownCommand extends Command {
    public ElevatorUpDownCommand() {
        addRequirements(Robot.instance.exampleSubsystem);
    }

    @Override
    public void initialize() {
        if (/*Check Up Button Pressed */) {
            // Increment Arm Up
        } else if (/*Check Down Button Pressed */) {
            // Increment Arm Down
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
