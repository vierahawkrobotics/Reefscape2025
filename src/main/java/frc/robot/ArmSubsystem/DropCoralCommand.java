package frc.robot.ArmSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class DropCoralCommand extends Command {
    private double startTime;
    public DropCoralCommand() {
        addRequirements(Robot.instance.exampleSubsystem);
    }

    @Override
    public void initialize() {}
    @Override
    public void execute() {
        ArmSubsystem.container.set(-1);
        ArmSubsystem.containerFollower.set(-1);
        startTime = Timer.getFPGATimestamp();
    }
    @Override
    public void end(boolean interrupted) {}
    @Override
    public boolean isFinished() {
        if (Timer.getFPGATimestamp()-startTime == ArmConstants.dropTime) {
            return true;
        } else {
            return false;
        }
    }
}
