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
        addRequirements(Robot.instance.drivetrain);
        this.x = x;
        this.y = y;
        this.r = r;
        this.stopButton = stopButton;
    }
    public DrivePoseBased(double x, double y, double r){
        addRequirements(Robot.instance.drivetrain);
        this.x = x;
        this.y = y;
        this.r = r;
        this.stopButton = () -> {return false;};
    }
    public DrivePoseBased(Pose2d pose, Supplier<Boolean> stopButton){
        addRequirements(Robot.instance.drivetrain);
        this.x = pose.getX();
        this.y = pose.getY();
        this.r = pose.getRotation().getRadians();
        this.stopButton = stopButton;
    }
    public DrivePoseBased(Pose2d pose){
        addRequirements(Robot.instance.drivetrain);
        this.x = pose.getX();
        this.y = pose.getY();
        this.r = pose.getRotation().getRadians();
        this.stopButton = () -> {return false;};
    }
    @Override
    public void initialize() {
        Robot.instance.drivetrain.setTargetPos(x, y);
        //Robot.instance.drivetrain.setTargetPosRot(r);
        System.out.println("Initialize run");
    }
    @Override
    public void execute(){
        // System.out.println("execute func");
    }

    @Override
    public void end(boolean interrupted){
        if (interrupted) System.out.println("interrupted");
        System.out.println("end");
    }
    @Override
    public boolean isFinished(){
        return (stopButton.get()) || 
        (Robot.instance.drivetrain.getIsPointReached() 
        && Robot.instance.drivetrain.getIsRotationReached()
        && Robot.instance.drivetrain.checkIsRobotStopped());
    }

}