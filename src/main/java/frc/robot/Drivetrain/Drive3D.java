package frc.robot.Drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

/*LEFT TO DO:
 * change the robot angle to be taken from the position subSystem
 */
public  class Drive3D extends Command {
  //TO DO: this can be changed later for area effects etc, note it must be meters/second
  double maxSpeed = 4;
  //TO DO: this should be taken from the position subsystem
  double robotAngle = 45.0;
  
  Supplier<Double> vx;
  Supplier<Double> vy;
  Supplier<Double> vr;

  public Drive3D(Supplier<Double> vxInput, Supplier<Double> vyInput, Supplier<Double> vrInput) {
    vx = vxInput;
    vy = vyInput;
    vr = vrInput;
    addRequirements(Robot.instance.drivetrain);
  }
  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    //apply input deadband, input squaring,

    double vxVal = vx.get();
    double vyVal = vy.get();
    double m = Math.sqrt(vxVal * vxVal + vyVal * vyVal);
  
    if(m <= DrivetrainConstants.inputDeadband) {
      vxVal = 0;
      vyVal = 0;
    }
    else {
      vxVal *= m;
      vyVal *= m;
    }

    Double vrVal;
    vrVal = vr.get();
    if(vrVal != null) Robot.instance.drivetrain.setTargetPosRot(vrVal);
    
    Robot.instance.drivetrain.setInputVel(vxVal, vyVal);
  }
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
