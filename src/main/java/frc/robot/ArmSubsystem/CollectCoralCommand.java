package frc.robot.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
public class CollectCoralCommand extends Command {

    public CollectCoralCommand() {
        addRequirements(Robot.instance.exampleSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
