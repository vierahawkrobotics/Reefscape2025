package frc.robot.Drivetrain;

import java.io.Console;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class DrivePoseBased extends Command{
    double x;
    double y;
    double r;
    Supplier<Boolean> bSupplier;
    /**
     * @author Giahna C
     * @param posX Desired x position, meters
     * @param posY Desired y position, meters
     * @param posR Desired robot rotation, radians
     * @param boolSupplier Button to quit go to
     */
    public DrivePoseBased(double posX, double posY, double posR, Supplier<Boolean> boolSupplier ){
        bSupplier = boolSupplier;
        x = posX;
        y = posY;
        r = posR;
    }
    /**
     * @author Giahna C
     * @param posX Desired x position, meters
     * @param posY Desired y position, meters
     * @param posR Desired robot rotation, radians
     */
    public DrivePoseBased(double posX, double posY, double posR){
        x = posX;
        y = posY;
        r = posR;
    }
    /**
     * @author Giahna C
     * @param pose Desired position containing an x, y, and rotation in meters & radians
     * @param boolSupplier Button to quit go to
     */
    public DrivePoseBased(Pose2d pose, Supplier<Boolean> boolSupplier){
        x = pose.getX();
        y = pose.getY();
        r = pose.getRotation().getRadians();
        bSupplier = boolSupplier;
    }
    @Override
    public void initialize() {
        Robot.instance.drivetrain.setTargetPos(x, y);
        Robot.instance.drivetrain.setTargetPosRot(r);
    }
    @Override
    public void execute(){
        
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("finished posirion stuff");
        if(interrupted) System.out.println("INTERP");
    }

    /**
     * @author Giahna C
     * @return Returns whether robot has arrived at the position or not
     */
    @Override
    public boolean isFinished() {
        if(bSupplier != null && bSupplier.get()) {
            return true;
        }
        if(Robot.instance.drivetrain.getIsPointReached() &&
        Robot.instance.drivetrain.getIsRotationReached() &&
        Robot.instance.drivetrain.checkIsRobotStopped()){
            System.out.println("Position Reached");
            return true;
        }
        return false;
    }
}
