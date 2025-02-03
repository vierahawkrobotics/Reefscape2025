package frc.robot.ArmSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class DropCoralCommand extends Command {
    public static double startTime;
    public DropCoralCommand() {
        addRequirements(Robot.instance.exampleSubsystem);
    }

    @Override
    public void initialize() {}
    @Override
    public void execute() {
        ArmSubsystem.setIntakeState(ArmSubsystem.IntakeState.Collect);
        startTime = Timer.getFPGATimestamp();
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
}
