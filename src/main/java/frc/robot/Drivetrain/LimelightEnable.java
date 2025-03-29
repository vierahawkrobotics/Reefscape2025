package frc.robot.Drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Components.LimelightComponent;
import frc.robot.Components.PositionComponent.PositionComponent;

/*LEFT TO DO:
 * change the robot angle to be taken from the position subSystem
 */
public  class LimelightEnable extends Command {

  boolean enableRotation;
  public LimelightEnable(boolean enableRotation) {
    this.enableRotation = enableRotation;
    addRequirements(Robot.instance.drivetrain);
  }
  @Override
  public void initialize() {
    LimelightComponent.EnableRotControls(enableRotation);
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
