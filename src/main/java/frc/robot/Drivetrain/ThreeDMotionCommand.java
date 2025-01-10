package frc.robot.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

/*LEFT TO DO:
 * debug rotation sensititivity
 * change the robot angle to be taken from the position subSystem]
 * Get motor type and add motors to Drivetrain.java, then apply
 *     moduleState[i] to its corresponding motor via .set
 * Add cosine compensation
 * Move the bulk of the command to Drivetrain.java
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
    
    //Create a ChassisSpeeds object from the field relative inputs and converts it to robot relative
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      -1*Drivetrain.getInstance().getControllerY(), 
      Drivetrain.getInstance().getControllerX(),
      Drivetrain.getInstance().getControllerRot(),
      Rotation2d.fromDegrees(robotAngle));
    
    //Create the individual moduleStates to be applied to the motors, then optimize them
    //TO DO: make sure .getDistance is outputing the correct encoder info in radians
    SwerveModuleState[] moduleStates = Drivetrain.kinematics.toSwerveModuleStates(speeds);
    moduleStates[0].optimize(new Rotation2d(Drivetrain.maxSwerveModules[0].turningEncoder.getPosition()));
    moduleStates[1].optimize(new Rotation2d(Drivetrain.maxSwerveModules[1].turningEncoder.getPosition()));
    moduleStates[2].optimize(new Rotation2d(Drivetrain.maxSwerveModules[2].turningEncoder.getPosition()));
    moduleStates[3].optimize(new Rotation2d(Drivetrain.maxSwerveModules[3].turningEncoder.getPosition()));
    
    Drivetrain.getInstance().setDesiredStates(moduleStates);
    
  }//*= cos(moduleStates[0].angle - moduleStates[0]'s current angle' */
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {return false;}
}
