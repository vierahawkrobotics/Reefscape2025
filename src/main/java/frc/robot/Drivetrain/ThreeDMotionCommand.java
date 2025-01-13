package frc.robot.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

/*LEFT TO DO:
 * debug rotation sensititivity
 * change the robot angle to be taken from the position subSystem]
 * Add cosine compensation
 */
public  class ThreeDMotionCommand extends Command {
  XboxController controller = new XboxController(DrivetrainConstants.usbPortController);
  //TO DO: this can be changed later for area effects etc, note it must be meters/second
  double maxSpeed = 4;
  //TO DO: this should be taken from the position subsystem
  double robotAngle = 45.0;

  //Code to make sure ThreeDMotionCommand only has one instance, please use getInstance to get an instance of this command
  private static final ThreeDMotionCommand threeDmotionCommand = new ThreeDMotionCommand();
  private ThreeDMotionCommand() {addRequirements(Drivetrain.getInstance());}
  public static ThreeDMotionCommand getInstance() {return threeDmotionCommand;}

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Drivetrain.getInstance().setDrivetrain(-1*Drivetrain.getInstance().getControllerY(), 
    Drivetrain.getInstance().getControllerX(),
    Drivetrain.getInstance().getControllerRot(),
    Rotation2d.fromDegrees(robotAngle));
    
  }//*= cos(moduleStates[0].angle - moduleStates[0]'s current angle' */
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {return false;}
}
