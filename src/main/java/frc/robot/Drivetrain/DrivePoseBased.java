package frc.robot.Drivetrain;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class DrivePoseBased extends Command{
    double x;
    double y;
    double r;
    Supplier<Boolean> stopButton;
    public DrivePoseBased(double x, double y, double r, Supplier<Boolean> stopButton){
        this.x = x;
        this.y = y;
        this.r = r;
        this.stopButton = stopButton;
    }
    public DrivePoseBased(double x, double y, double r){
        this.x = x;
        this.y = y;
        this.r = r;
        this.stopButton = () -> {return false;};
    }
    public DrivePoseBased(Pose2d pose, Supplier<Boolean> stopButton){
        this.x = pose.getX();
        this.y = pose.getY();
        this.r = pose.getRotation().getRadians();
        this.stopButton = stopButton;
    }
    public DrivePoseBased(Pose2d pose){
        this.x = pose.getX();
        this.y = pose.getY();
        this.r = pose.getRotation().getRadians();
        this.stopButton = () -> {return false;};
    }
    @Override
    public void initialize() {
        Robot.instance.drivetrain.setTargetPos(x, y);
        Robot.instance.drivetrain.setTargetPosRot(r);
    }
    @Override
    public void execute(){}

    @Override
    public void end(boolean interrupted){}
    @Override
    public boolean isFinished(){
        return (stopButton.get()) || 
        (Robot.instance.drivetrain.getIsPointReached() 
        && Robot.instance.drivetrain.getIsRotationReached()
        && Robot.instance.drivetrain.checkIsRobotStopped());
    }

}