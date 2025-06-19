package frc.robot.Drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Components.PositionComponent.PositionComponent;

public class GoToPoint extends Command{
    Pose2d targetPosition;
    double startTime = 0;
    Supplier<Boolean> stopButton;
    boolean checkIsRobotStopped;

    TrapezoidProfile.State xStartState;
    TrapezoidProfile.State xEndState;
    TrapezoidProfile.State yStartState;
    TrapezoidProfile.State yEndState;
    TrapezoidProfile.State rotationStartState;
    TrapezoidProfile.State rotationEndState;
    
    /**
     * Constructor for a GoToPoint object
     * @author Giahna C.
     * @param x The desired position for the bot along the x-axis (uses NWU coords)
     * @param y The desired position for the bot along the y-axis (uses NWU coords)
     * @param rotation The desired angle for the bot (uses NWU coords)
     * @param stopButton A supplier that will stop the command if true.
     * @param checkIsRobotStopped If this value is true the method will check if the bot has a velocity of 0 before considering the command complete.
     * If false the command will be over the second the bot is at the point.
     */
    public GoToPoint(double x, double y, double rotation, Supplier<Boolean> stopButton, boolean checkIsRobotStopped){
        targetPosition = new Pose2d(x, y, new Rotation2d(rotation));
        this.stopButton = stopButton;
        this.checkIsRobotStopped = checkIsRobotStopped;
        addRequirements(Drivetrain.getInstance());
    }
    /**
     *  Constructor for a GoToPoint object
     * @author Giahna C.
     * @param targetPosition The position the robot should be in at the end, contains x, y, and rotation. (uses NWU coords)
     * @param stopButton A supplier that will stop the command if true.
     * @param checkIsRobotStopped If this value is true the method will check if the bot has a velocity of 0 before considering the command complete.
     * If false the command will be over the second the bot is at the point.
     */
    public GoToPoint(Pose2d targetPosition, Supplier<Boolean> stopButton, boolean checkIsRobotStopped){
        this.targetPosition = targetPosition;
        this.stopButton = stopButton;
        this.checkIsRobotStopped = checkIsRobotStopped;
        addRequirements(Drivetrain.getInstance());
    }
    /**
     *  Constructor for a GoToPoint object
     * @author Giahna C.
     * @param x The desired position for the bot along the x-axis (uses NWU coords)
     * @param y The desired position for the bot along the y-axis (uses NWU coords)
     * @param rotation The desired angle for the bot (uses NWU coords)
     * @param checkIsRobotStopped If this value is true the method will check if the bot has a velocity of 0 before considering the command complete.
     * If false the command will be over the second the bot is at the point.
     */
    public GoToPoint(double x, double y, double rotation, boolean checkIsRobotStopped){
        targetPosition = new Pose2d(x, y, new Rotation2d(rotation));
        this.stopButton = () -> {return false;};
        this.checkIsRobotStopped = checkIsRobotStopped;
        addRequirements(Drivetrain.getInstance());
    }
    /**
     *  Constructor for a GoToPoint object
     * @author Giahna C.
     * @param targetPosition The position the robot should be in at the end, contains x, y, and rotation. (uses NWU coords)
     * @param checkIsRobotStopped If this value is true the method will check if the bot has a velocity of 0 before considering the command complete.
     * If false the command will be over the second the bot is at the point.
     */
    public GoToPoint(Pose2d targetPosition, boolean checkIsRobotStopped){
        this.targetPosition = targetPosition;
        this.stopButton = () -> {return false;};
        this.checkIsRobotStopped = checkIsRobotStopped;
        addRequirements(Drivetrain.getInstance());
    }

    @Override
    public void initialize(){
        startTime = Timer.getFPGATimestamp();
        //sets the overall states for the profiles
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
        TrapezoidProfile.State xSetpoint = DrivetrainConstants.xProfile.calculate((Timer.getFPGATimestamp() - startTime), xStartState, xEndState);
        TrapezoidProfile.State ySetpoint = DrivetrainConstants.yProfile.calculate((Timer.getFPGATimestamp() - startTime), yStartState, yEndState);
        TrapezoidProfile.State rotSetpoint = DrivetrainConstants.rotProfile.calculate((Timer.getFPGATimestamp() - startTime), rotationStartState, rotationEndState);
        Drivetrain.getInstance().setDrivePositionPIDs(xSetpoint, ySetpoint, rotSetpoint);
    }

    @Override
    public void end(boolean interrupted){
        Drivetrain.getInstance().setVelocityPIDs(0,0,0, false,false);
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

    /**
     * Gets the distance between the robot and the point the robot is going to that was set in the constructor
     * @author Giahna C.
     * @return the distance
     */
    public double getDistanceFromPoint(){
        Pose2d curPos = PositionComponent.getRobotPose();
        return MiscMathFunctions.distance(curPos.getX(), targetPosition.getX(), curPos.getX(), targetPosition.getY());
    }

    /**
     * Gets the difference in radians between the robot's rotation and the rotation the robot was set to in the constructor
     * @author Giahna C.
     * @return the differnce
     */
    public double getDifferenceFromAngle(){
        double currentAngle = PositionComponent.getRobotPose().getRotation().getRadians()*-1;
        return MiscMathFunctions.mod(targetPosition.getRotation().getRadians() - currentAngle -Math.PI, 2*Math.PI) - Math.PI;
    }

    /**
     * Gets the position this command is set to go to
     * @author Giahna C. 
     * @return the target position
     */
    public Pose2d getTargetPosition(){
        return targetPosition;
    }
}
