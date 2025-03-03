package frc.robot.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ElevatorUpDownCommand extends Command {
    private boolean move;
    public ElevatorUpDownCommand(boolean move) { // true = up, false = down
        addRequirements(Robot.instance.armSubsystem);
        this.move = move;
    }

    @Override
    public void initialize() {
        double height = Robot.instance.armSubsystem.getTargetHeight();
        if (move) { // Check go up
            System.out.println("up");
            System.out.println("height: " + height);
            if(height == ArmConstants.HeightState.CoralLow.getHeight() || height == ArmConstants.HeightState.AlgaeLow.getHeight()) {
                Robot.instance.armSubsystem.setHeightState(ArmConstants.HeightState.CoralHigh);
            } else if (height == ArmConstants.HeightState.Collect.getHeight()) {
                Robot.instance.armSubsystem.setHeightState(ArmConstants.HeightState.CoralLow);
            } else { // Error or Ground State
                Robot.instance.armSubsystem.setHeightState(ArmConstants.HeightState.Collect);
            }
            System.out.println("next: " + Robot.instance.armSubsystem.getTargetHeight());
        } else if (!move) { // Check go down
            if (height == ArmConstants.HeightState.CoralHigh.getHeight() || height == ArmConstants.HeightState.AlgaeHigh.getHeight()) {
                Robot.instance.armSubsystem.setHeightState(ArmConstants.HeightState.CoralLow);
            } else if (height == ArmConstants.HeightState.CoralLow.getHeight()) {
                Robot.instance.armSubsystem.setHeightState(ArmConstants.HeightState.Collect);
            } else { // Error or Collect State
                Robot.instance.armSubsystem.setHeightState(ArmConstants.HeightState.Ground);
            }
        }
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {}
    
    @Override
    public boolean isFinished() {
        return true;
    }
}