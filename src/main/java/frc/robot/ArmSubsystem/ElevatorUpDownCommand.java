package frc.robot.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

enum heightState {
    Ground,
    Low,
    High
}

public class ElevatorUpDownCommand extends Command {
    public heightState state = heightState.Ground;
    private boolean up;
    private boolean down;
    public ElevatorUpDownCommand(boolean up, boolean down) {
        addRequirements(Robot.instance.exampleSubsystem);
        this.up = up;
        this.down = down;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (up) { // Check go up
            switch (state) {
                case Ground: // Go up and set state to Low
                    ArmSubsystem.SetTargetHeight(ArmConstants.low);
                    if (ArmSubsystem.AtTargetHeight()) {
                        state = heightState.Low;
                    }
                    break;
                case Low: // Go up and set state to High
                    ArmSubsystem.SetTargetHeight(ArmConstants.high);
                    if (ArmSubsystem.AtTargetHeight()) {
                        state = heightState.High;
                    }
                    break;
                case High: // Do nothing
                    break;
            }
        } else if (down) { // Check go down
            switch (state) {
                case Ground: // Do nothing
                    break;
                case Low: // Go down and set state to Ground
                    ArmSubsystem.SetTargetHeight(ArmConstants.ground);
                    if (ArmSubsystem.AtTargetHeight()) {
                        state = heightState.Ground;
                    }
                    break;
                case High: // Go down and set state to Low
                    ArmSubsystem.SetTargetHeight(ArmConstants.low);
                    if (ArmSubsystem.AtTargetHeight()) {
                        state = heightState.Low;
                    }
                    break;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {}
    
    @Override
    public boolean isFinished() {
        return false;
    }
}