package frc.robot.Drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Components.AreaEffects.AreaEffectsHandler;
import frc.robot.Components.PositionComponent.PositionComponent;

public class AutoTurning extends Command{
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
     * used to calculate the desired rotation for the robot. If an area effect is in a certain area, the robot will turn based on that area effect.
     * @author Giahna C.
     * @param vx The velocity for the bot along the x-axis. [-1,1]
     * @param vy The velocity for the bot along the x-axis. [-1,1] 
     * @param rX The x position of the right joystick.
     * @param rY The y position of the right joystick. Make sure to invert this from the joystick.
     */
    public AutoTurning(Supplier<Double> vx, Supplier<Double> vy, Supplier<Double> rX, Supplier<Double> rY){
        this.vx = vx;
        this.vy = vy;
        this.rX = rX;
        this.rY = rY;
        addRequirements(Drivetrain.getInstance());
    }
    @Override
    public void execute(){
        lastAngle = PositionComponent.getRobotPose().getRotation().getRadians();
        //input squaring
        velX = Math.signum(vx.get())*Math.pow(vx.get(), 2);
        velY = Math.signum(vy.get())*Math.pow(vy.get(), 2);
        rotX = Math.signum(rX.get())*Math.pow(rX.get(),2);
        rotY = Math.signum(rY.get())*Math.pow(rY.get(),2);
        //input deadband for driving velocity
        if (Math.hypot(velX,velY) < DrivetrainConstants.inputDeadband){
            velX = 0;
            velY = 0;
        }
        //check for area pose and apply that if there's one
        Pose2d areaPose = AreaEffectsHandler.getTargetPose();
        if (areaPose != null){
            targetAngle = areaPose.getRotation().getRadians();
        }
        //if there's no area pose check that the rotation is in the input deadband
        //if it is set the rotation to the last angle the robot was at.
        else if (Math.hypot(rotX, rotY) < DrivetrainConstants.inputDeadband){
            targetAngle = lastAngle; //if this bugs try setting velR to 0 later instead.
        }
        //if the rotation inputs are outside the deadband, get the angle that the joystick is at
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
