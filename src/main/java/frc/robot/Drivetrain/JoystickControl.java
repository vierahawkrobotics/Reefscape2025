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
     */
    public JoystickControl(Supplier<Double> vx, Supplier<Double> vy, Supplier<Double> rX){
        this.vx = vx;
        this.vy = vy;
        this.rX = rX;
        addRequirements(Drivetrain.getInstance());
    }
    @Override
    public void execute(){
        //input squaring
        velX = Math.signum(vx.get())*Math.pow(MathUtil.clamp(vx.get(), -1, 1), 2);
        velY = Math.signum(vy.get())*Math.pow(MathUtil.clamp(vy.get(), -1, 1), 2);
        velR = Math.signum(rX.get())*Math.pow(MathUtil.clamp(rX.get(), -1, 1),2);
        // rotY = Math.signum(rY.get())*Math.pow(MathUtil.clamp(rY.get(), -1, 1),2);
        //input deadband
        if (Math.hypot(velX,velY) < DrivetrainConstants.inputDeadband){
            velX = 0;
            velY = 0;
        }
        if (Math.abs(velR) < DrivetrainConstants.inputDeadband){
            velR = 0;
        }
        Drivetrain.getInstance().setVelocityPIDs(velX, velY, velR, true, true);
    }
    @Override
    public void end(boolean interrupted) {
        System.out.println("test");
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
