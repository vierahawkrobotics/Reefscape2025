package frc.robot.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
public class ElevatorUpDownAction extends Command {
    private float targetHeight;
    public ElevatorUpDownAction(float targetHeight) {
        addRequirements(Robot.instance.armSubsystem);
        this.targetHeight = targetHeight;
    }

    @Override
    public void initialize() {
        Robot.instance.armSubsystem.setHeight(targetHeight);
    }
    @Override
    public void execute() {
        
    }
    @Override
    public void end(boolean interrupted) {}
    @Override
    public boolean isFinished() {
        return Robot.instance.armSubsystem.isAtTargetHeight();
    }
}
