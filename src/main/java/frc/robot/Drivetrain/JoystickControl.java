package frc.robot.Drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Components.PositionComponent;

public class JoystickControl extends Command{
    //make sure this is the default command for drivetrain.
    Supplier<Double> vx;
    Supplier<Double> vy;
    Supplier<Double> rX;
    Supplier<Double> rY;
    double velX;
    double velY;
    double rotX;
    double rotY;
    double targetAngle;
    double lastAngle;
    double velR;

    /**
     * This method assumes inputs are in NWU. Joystick input is in NED. Vx and Vy will be the velocities used for the bot. rX and rY will be
     * used to calculate the desired rotation for the robot.
     * @author Giahna C.
     * @param vx The velocity for the bot along the x-axis. [-1,1]
     * @param vy The velocity for the bot along the x-axis. [-1,1] 
     * @param rX The x position of the right joystick.
     * @param rY The y position of the right joystick. Make sure to invert this from the joystick.
     */
    public JoystickControl(Supplier<Double> vx, Supplier<Double> vy, Supplier<Double> rX, Supplier<Double> rY){
        this.vx = vx;
        this.vy = vy;
        this.rX = rX;
        this.rY = rY;
        addRequirements(Drivetrain.getInstance());
    }
    @Override
    public void execute(){
        lastAngle = PositionComponent.getPose2d().getRotation().getRadians();
        //input squaring
        velX = Math.signum(vx.get())*Math.pow(MathUtil.clamp(vx.get(), -1, 1), 2);
        velY = Math.signum(vy.get())*Math.pow(MathUtil.clamp(vy.get(), -1, 1), 2);
        rotX = Math.signum(rX.get())*Math.pow(MathUtil.clamp(rX.get(), -1, 1),2);
        rotY = Math.signum(rY.get())*Math.pow(MathUtil.clamp(rY.get(), -1, 1),2);
        //input deadband
        if (Math.hypot(velX,velY) < DrivetrainConstants.inputDeadband){
            velX = 0;
            velY = 0;
        }
        if (Math.hypot(rotX, rotY) < DrivetrainConstants.inputDeadband){
            targetAngle = lastAngle; //if this bugs try setting velR to 0 later instead.
        }
        else{
            targetAngle = Math.atan2(rotY, rotX);
        }
         //find the velocity for rotation, this method returns the value in radians/sec.
        velR = Drivetrain.getInstance().getVelocityToSetTargetAngle(targetAngle);
        Drivetrain.getInstance().setVelocityPIDs(velX, velY, velR, true, false);
    }
    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished(){
        return false;
    }
}
