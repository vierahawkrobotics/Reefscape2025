package frc.robot.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

/*LEFT TO DO:
 * debug rotation sensititivity
 * change the robot angle to be taken from the position subSystem]
 * Add cosine compensation
 */
//rename to drive3dCommand
public  class ThreeDMotionCommand extends Command {
  XboxController controller = new XboxController(DrivetrainConstants.usbPortController);
  //TO DO: this can be changed later for area effects etc, note it must be meters/second
  double maxSpeed = 4;
  //TO DO: this should be taken from the position subsystem
  double robotAngle = 45.0;

  //add parameters and supplier<Float> vx, vy, vr
  private ThreeDMotionCommand() {
    addRequirements(Robot.instance.drivetrain);
  }
  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    //drivetrain.setDrive3d(vx.get(),vy.get(),vr.get())
  }
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
