package frc.robot.ArmSubsystem;

import frc.robot.Components.CANdle.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class CollectCoralCommand extends Command {
    public CollectCoralCommand() {
        addRequirements(Robot.instance.armSubsystem);
    }

    @Override
    public void initialize() {
        Robot.instance.armSubsystem.setHeightState(ArmConstants.HeightState.Collect);
        Robot.instance.armSubsystem.setIntakeState(ArmConstants.IntakeState.Collect);
        CANdleController.setState(CANdleConstants.RobotStates.Intaking);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        Robot.instance.armSubsystem.setHeightState(ArmConstants.HeightState.Ground);
        Robot.instance.armSubsystem.setIntakeState(ArmConstants.IntakeState.Rest);
    }

    @Override
    public boolean isFinished() {
        return Robot.instance.armSubsystem.getIntakeState() == ArmConstants.IntakeState.Rest;
    }
}
