package frc.robot.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ElevatorSetHeightCommand extends Command {
    private ArmConstants.HeightState state;
    /**
     * @param move true = high, false = low
     */
    public ElevatorSetHeightCommand(ArmConstants.HeightState state) {
        addRequirements(Robot.instance.armSubsystem);
        this.state = state;
    }

    @Override
    public void initialize() {
        Robot.instance.armSubsystem.setHeightState(state);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}
    
    @Override
    public boolean isFinished() {
        return Robot.instance.armSubsystem.AtTargetHeight();
    }
}