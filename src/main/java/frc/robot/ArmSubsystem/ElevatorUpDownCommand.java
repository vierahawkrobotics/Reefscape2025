package frc.robot.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ElevatorUpDownCommand extends Command {
    private boolean up;
    private boolean down;
    public ElevatorUpDownCommand(boolean up, boolean down) {
        addRequirements(Robot.instance.armSubsystem);
        this.up = up;
        this.down = down;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double height = Robot.instance.armSubsystem.getTargetHeight();
        if (up) { // Check go up
            if(height == ArmConstants.HeightState.CoralHigh.getHeight()) {
                Robot.instance.armSubsystem.setHeightState(ArmConstants.HeightState.AlgaeHigh);
            }
            else if(height == ArmConstants.HeightState.AlgaeLow.getHeight()) {
                Robot.instance.armSubsystem.setHeightState(ArmConstants.HeightState.CoralHigh);
            }
            else if(height == ArmConstants.HeightState.CoralLow.getHeight()) {
                Robot.instance.armSubsystem.setHeightState(ArmConstants.HeightState.AlgaeLow);
            }
            else { //error or ground
                Robot.instance.armSubsystem.setHeightState(ArmConstants.HeightState.CoralLow);
            }
        } else if (down) { // Check go down
            if(height == ArmConstants.HeightState.CoralLow.getHeight()) {
                Robot.instance.armSubsystem.setHeightState(ArmConstants.HeightState.Ground);
            }
            else if(height == ArmConstants.HeightState.AlgaeLow.getHeight()) {
                Robot.instance.armSubsystem.setHeightState(ArmConstants.HeightState.CoralLow);
            }
            else if(height == ArmConstants.HeightState.CoralHigh.getHeight()) {
                Robot.instance.armSubsystem.setHeightState(ArmConstants.HeightState.AlgaeLow);
            }
            else { //error or algaehigh
                Robot.instance.armSubsystem.setHeightState(ArmConstants.HeightState.AlgaeHigh);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {}
    
    @Override
    public boolean isFinished() {
        return true;
    }
}