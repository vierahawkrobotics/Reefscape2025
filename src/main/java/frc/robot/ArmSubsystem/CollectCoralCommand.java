package frc.robot.ArmSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
// import frc.robot.Components.CANdleComponent.CANdleConstants;
// import frc.robot.Components.CANdleComponent.CANdleController;

public class CollectCoralCommand extends Command {
    private Supplier<Boolean> interrupted;
    public CollectCoralCommand(Supplier<Boolean> interrupted) {
        addRequirements(Robot.instance.armSubsystem);
        this.interrupted = interrupted;
    }

    @Override
    public void initialize() {
        Robot.instance.armSubsystem.setHeightState(ArmConstants.HeightState.Collect);
        Robot.instance.armSubsystem.setIntakeState(ArmConstants.IntakeState.Collect);
        // CANdleController.setState(CANdleConstants.RobotStates.Intaking);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        Robot.instance.armSubsystem.setHeightState(ArmConstants.HeightState.Ground);
        Robot.instance.armSubsystem.setIntakeState(ArmConstants.IntakeState.Rest);
        // CANdleController.setState(CANdleConstants.RobotStates.Idle);
    }

    @Override
    public boolean isFinished() {
        return Robot.instance.armSubsystem.getIntakeState() == ArmConstants.IntakeState.Rest || interrupted.get();
    }
}
