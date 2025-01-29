package frc.robot.ArmSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
public class CollectCoralCommand extends Command {
    private double startTime;
    public CollectCoralCommand() {
        addRequirements(Robot.instance.exampleSubsystem);
    }

    @Override
    public void initialize() {
        ArmSubsystem.container.set(1);
        ArmSubsystem.containerFollower.set(1);
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        if (ArmSubsystem.container.getForwardLimitSwitch().isPressed()) {
            return true;
        } else {
            return false;
        }
    }
}
