package frc.robot.Components.CANdleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
public class CANdleStateChangeCommand extends Command {
    private CANdleConstants.RobotStates state;
    public CANdleStateChangeCommand(CANdleConstants.RobotStates state) {
        addRequirements(Robot.instance.exampleSubsystem);
        this.state = state;
    }

    @Override
    public void initialize() {
        Robot.instance.candle.setState(this.state);
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
