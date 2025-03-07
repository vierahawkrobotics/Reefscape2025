package frc.robot.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Components.CANdleComponent.CANdleConstants;
import frc.robot.Components.CANdleComponent.CANdleController;

public class ElevatorMovementCommand extends Command {
    private int move;
    private boolean algae;
    private boolean reset;
    /**
     * @param move 2 = up, 1 = down, 0 = skip
     * @param algae true = change, false = skip
     * @param reset true = reset, false = skip
     */
    public ElevatorMovementCommand(int move, boolean algae, boolean reset) {
        addRequirements(Robot.instance.armSubsystem);
        this.move = move;
        this.algae = algae;
        this.reset = reset;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double height = Robot.instance.armSubsystem.getTargetHeight();
        if (this.move == 2) { // Check go up
            if(height == ArmConstants.HeightState.CoralLow.getHeight()) {
                Robot.instance.armSubsystem.setHeightState(ArmConstants.HeightState.CoralHigh);
            } else { // Error, Ground, or aglae height
                Robot.instance.armSubsystem.setHeightState(ArmConstants.HeightState.CoralLow);
            }
            Robot.instance.armSubsystem.setAlgaeMotorSpeed(ArmConstants.AlgaeMotorState.Inactive);
        } else if (this.move == 1) { // Check go down
            if (height == ArmConstants.HeightState.CoralHigh.getHeight()) {
                Robot.instance.armSubsystem.setHeightState(ArmConstants.HeightState.CoralLow);
            } else { // Error, CoralLow, or algae height
                Robot.instance.armSubsystem.setHeightState(ArmConstants.HeightState.Ground);
            }
            Robot.instance.armSubsystem.setAlgaeMotorSpeed(ArmConstants.AlgaeMotorState.Inactive);
        } else if (algae) { // Cycle algae
            if (height == ArmConstants.HeightState.AlgaeLow.getHeight()) {
                Robot.instance.armSubsystem.setHeightState(ArmConstants.HeightState.AlgaeHigh);
            } else { // Not at algae height or at AlgaeHigh
                Robot.instance.armSubsystem.setHeightState(ArmConstants.HeightState.AlgaeLow);
            }
            Robot.instance.armSubsystem.setAlgaeMotorSpeed(ArmConstants.AlgaeMotorState.Active);
        } else if (reset) { // Reset to Ground
            Robot.instance.armSubsystem.setHeightState(ArmConstants.HeightState.Ground);
            Robot.instance.armSubsystem.setAlgaeMotorSpeed(ArmConstants.AlgaeMotorState.Inactive);
        }
        CANdleController.setState(CANdleConstants.RobotStates.ElevatorMoving);
    }

    @Override
    public void end(boolean interrupted) {
        CANdleController.setState(CANdleConstants.RobotStates.Idle);
    }
    
    @Override
    public boolean isFinished() {
        return Robot.instance.armSubsystem.AtTargetHeight();
    }
}