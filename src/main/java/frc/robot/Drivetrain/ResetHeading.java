package frc.robot.Drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Components.PositionComponent.PositionComponent;

/*LEFT TO DO:
 * change the robot angle to be taken from the position subSystem
 */
public  class ResetHeading extends Command {

  public ResetHeading() {
    addRequirements(Robot.instance.drivetrain);
  }
  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
      PositionComponent.ResetDirection();
  }
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
