package frc.robot.Drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Components.LimelightComponent;
import frc.robot.Components.PositionComponent.PositionComponent;

/*LEFT TO DO:
 * change the robot angle to be taken from the position subSystem
 */
public  class SetSpeedCommand extends Command {

  double speed;
  public SetSpeedCommand(double speed) {
    this.speed = speed;
  }
  @Override
  public void initialize() {
    DrivetrainConstants.defaultMaxSpeed = speed;
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
