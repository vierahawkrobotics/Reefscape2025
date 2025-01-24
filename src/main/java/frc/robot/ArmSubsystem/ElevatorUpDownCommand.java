
package frc.robot.ArmSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

enum heightState {
    Ground,
    Low,
    High
}

public class ElevatorUpDownCommand extends Command {
    public XboxController control = new XboxController(0); // temp
    public heightState
    public ElevatorUpDownCommand() {
        addRequirements(Robot.instance.exampleSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if ()
        // Button for going up

        // Button for going down
    }

    @Override
    public void end(boolean interrupted) {}
    
    @Override
    public boolean isFinished() {
        return false;
    }
}