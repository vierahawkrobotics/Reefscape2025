package frc.robot.Drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class DrivePoseBased extends Command{
    boolean bool;
    /**
     * @author Giahna C
     * @param posX Desired x position, meters
     * @param posY Desired y position, meters
     * @param posR Desired robot rotation, radians
     * @param boolSupplier Button to quit go to
     */
    public DrivePoseBased(double posX, double posY, double posR, Supplier<Boolean> boolSupplier){
        addRequirements(Robot.instance.drivetrain);
        bool = boolSupplier.get();
        Robot.instance.drivetrain.setTargetPos(posX, posY);
        Robot.instance.drivetrain.setTargetPosRot(posR);
        Robot.instance.drivetrain.setIsPointReached(false);
    }
    /**
     * @author Giahna C
     * @param pose Desired position containing an x, y, and rotation in meters & radians
     * @param boolSupplier Button to quit go to
     */
    public DrivePoseBased(Pose2d pose, Supplier<Boolean> boolSupplier){
        addRequirements(Robot.instance.drivetrain);
        bool = boolSupplier.get();
        Robot.instance.drivetrain.setTargetPos(pose.getX(), pose.getY());
        Robot.instance.drivetrain.setTargetPosRot(pose.getRotation().getRadians());
        Robot.instance.drivetrain.setIsPointReached(false);
    }
    @Override
    public void execute(){
    }

    @Override
    public void end(boolean interrupted) {}

    /**
     * @author Giahna C
     * @return Returns whether robot has arrived at the position or not
     */
    @Override
    public boolean isFinished() {
        if(bool) return true;
        if(Robot.instance.drivetrain.distance < DrivetrainConstants.validRange && Robot.instance.drivetrain.rotDistance < DrivetrainConstants.validRotDiff && Robot.instance.drivetrain.checkIsRobotStopped()){
            Robot.instance.drivetrain.setIsPointReached(true);
            return true;
        }
        return false;
    }
}
