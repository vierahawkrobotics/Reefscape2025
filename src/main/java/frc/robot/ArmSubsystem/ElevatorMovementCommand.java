package frc.robot.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ElevatorMovementCommand extends Command {
    private int move;
    private boolean reset;
    /**
     * @param move 2 = up, 1 = down, 0 = skip
     * @param reset true = reset, false = skip
     */
    public ElevatorMovementCommand(int move, boolean reset) {
        addRequirements(Robot.instance.armSubsystem);
        this.move = move;
        this.reset = reset;
    }

    @Override
    public void initialize() {
        double height = Robot.instance.armSubsystem.getTargetHeight();
        if (this.move == 2) { // Check go up
            System.out.println("Up");
            if(height == ArmConstants.HeightState.Ground.getHeight()) {
                Robot.instance.armSubsystem.setHeightState(ArmConstants.HeightState.Coral);
            }
        } else if (this.move == 1) { // Check go down
            System.out.println("Down");
            if (height == ArmConstants.HeightState.Coral.getHeight() || height == ArmConstants.maxHeight) {
                Robot.instance.armSubsystem.setHeightState(ArmConstants.HeightState.Ground);
            }
        } else if (this.reset) { // Reset to Ground
            System.out.println("Reset");
            Robot.instance.armSubsystem.setHeightState(ArmConstants.HeightState.Ground);
        }
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