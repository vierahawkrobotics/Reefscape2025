package frc.robot.Drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Components.PositionComponent.PositionComponent;

public class GoToPoint extends Command{
    Pose2d targetPosition;
    double timeElapsed;
    Supplier<Boolean> stopButton;
    boolean checkIsRobotStopped;

    TrapezoidProfile.State xStartState;
    TrapezoidProfile.State xEndState;
    TrapezoidProfile.State yStartState;
    TrapezoidProfile.State yEndState;
    TrapezoidProfile.State rotationStartState;
    TrapezoidProfile.State rotationEndState;
    

    public GoToPoint(double x, double y, double rotation, Supplier<Boolean> stopButton, boolean checkIsRobotStopped){
        targetPosition = new Pose2d(x, y, new Rotation2d(rotation));
        this.stopButton = stopButton;
        this.checkIsRobotStopped = checkIsRobotStopped;
        addRequirements(Drivetrain.getInstance());
    }
    public GoToPoint(Pose2d targetPosition, Supplier<Boolean> stopButton, boolean checkIsRobotStopped){
        this.targetPosition = targetPosition;
        this.stopButton = stopButton;
        this.checkIsRobotStopped = checkIsRobotStopped;
        addRequirements(Drivetrain.getInstance());
    }
    public GoToPoint(double x, double y, double rotation, boolean checkIsRobotStopped){
        targetPosition = new Pose2d(x, y, new Rotation2d(rotation));
        this.stopButton = () -> {return false;};
        this.checkIsRobotStopped = checkIsRobotStopped;
        addRequirements(Drivetrain.getInstance());
    }
    public GoToPoint(Pose2d targetPosition, boolean checkIsRobotStopped){
        this.targetPosition = targetPosition;
        this.stopButton = () -> {return false;};
        this.checkIsRobotStopped = checkIsRobotStopped;
        addRequirements(Drivetrain.getInstance());
    }

    @Override
    public void initialize(){
        //TODO: should timeElapsed be replaced by something that's more accurate(incases of loop overrun) but takes longer to compute?
        timeElapsed = 0;
        //get the overall states for the profiles
        xStartState = new TrapezoidProfile.State(PositionComponent.getRobotPose().getX(), PositionComponent.getChassisSpeeds().vxMetersPerSecond);
        xEndState = new TrapezoidProfile.State(targetPosition.getX(), 0);
        yStartState = new TrapezoidProfile.State(PositionComponent.getRobotPose().getY(), PositionComponent.getChassisSpeeds().vyMetersPerSecond);
        yEndState = new TrapezoidProfile.State(targetPosition.getY(), 0);
        rotationStartState = new TrapezoidProfile.State(PositionComponent.getRobotPose().getRotation().getRadians(), PositionComponent.getChassisSpeeds().omegaRadiansPerSecond);
        rotationEndState = new TrapezoidProfile.State(targetPosition.getRotation().getRadians(), 0);
    }

    @Override
    public void execute(){
        //get the current setpoints for the profiles
        TrapezoidProfile.State xSetpoint = DrivetrainConstants.xProfile.calculate(timeElapsed, xStartState, xEndState);
        TrapezoidProfile.State ySetpoint = DrivetrainConstants.yProfile.calculate(timeElapsed, yStartState, yEndState);
        TrapezoidProfile.State rotSetpoint = DrivetrainConstants.rotProfile.calculate(timeElapsed, rotationStartState, rotationEndState);
        timeElapsed += 0.02; // Assuming this command is run every 20ms
        Drivetrain.getInstance().setPositionPIDs(xSetpoint, ySetpoint, rotSetpoint);
    }

    @Override
    public void end(boolean interrupted){
        Drivetrain.getInstance().setVelocityPIDs(0,0,0, false);
    }

    @Override
    public boolean isFinished(){
        boolean isRobotStopped = checkIsRobotStopped? Drivetrain.getInstance().isRobotStopped(): true;
        return stopButton.get() || (
         getDistanceFromPoint() < DrivetrainConstants.atPointTarget &&
         getDifferenceFromAngle() < DrivetrainConstants.atRotTarget &&
         isRobotStopped
        );
    }

    public double getDistanceFromPoint(){
        Pose2d curPos = PositionComponent.getRobotPose();
        // return Math.sqrt(
        //     Math.pow(PositionComponent.getRobotPose().getX() - targetPosition.getX(),2) +
        //     Math.pow(PositionComponent.getRobotPose().getY() - targetPosition.getY(),2)
        // );
        return MiscMathFunctions.distance(curPos.getX(), targetPosition.getX(), curPos.getX(), targetPosition.getY());
    }

    public double getDifferenceFromAngle(){
        double currentAngle = PositionComponent.getRobotPose().getRotation().getRadians()*-1;
        return MiscMathFunctions.mod(targetPosition.getRotation().getRadians() - currentAngle -Math.PI, 2*Math.PI) - Math.PI;
    }
}
