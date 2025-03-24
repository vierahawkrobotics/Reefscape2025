package frc.robot.Components.PositionComponent;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Components.LimelightComponent;
import frc.robot.Components.LimelightComponent.PoseWithTimestamp;
import frc.robot.Components.PositionComponent.PositionComponentSettings.*;
import frc.robot.Components.PositionTools.PositionTools;
import frc.robot.Drivetrain.Drivetrain;

/**
 * The component of the robot that controls positioning using 
 * {@link LimelightComponent Limelight} and {@link SwerveDrivePoseEstimator WPILib Pose Estimator}
 * @author Darren Ringer
 * @author Richard Wright
 */
public class PositionComponent {
    private static SwerveDrivePoseEstimator poseEstimator;
    private static SwerveModulePosition[] wheelPositions;
    private static AHRS gyroObject;
    private static Pose2d[] lastPose = new Pose2d[2];
    private static long[] lastTimestamp = new long[2];
    private static PositionComponent instance;
    public static double gyroOffset = 0;
    private static double currentRad = 0;
    Pose2d initPose;

    private PositionComponent(Pose2d initialPose) {
        this.initPose = initialPose;
        gyroObject = new AHRS(NavXComType.kMXP_SPI);
        InitPose();
    }
    public static PositionComponent initialize(Pose2d initialPose){
        instance  = new PositionComponent(initialPose);
        return instance;
    }
    public static PositionComponent getInstance(){
        if(instance == null){
            initialize(new Pose2d());
            (new Alert("Error: Trying to get instance of PositionComponent before initialization. Assuming no offset.", AlertType.kError)).set(true);
        }
        return instance;
    }
    public void InitPose() {
        while(gyroObject.isCalibrating()) { Thread.yield();}
        System.out.println(Math.toRadians(gyroObject.getAngle()));
        gyroOffset = gyroObject.getRotation2d().getRadians() + initPose.getRotation().getRadians();
        poseEstimator = new SwerveDrivePoseEstimator(Drivetrain.kinematics, initPose.getRotation(), Drivetrain.getSwerveModulePositions(), initPose); // Fix kinematics and modulePositions parameter
        lastPose[0] = poseEstimator.getEstimatedPosition();
        lastPose[1] = poseEstimator.getEstimatedPosition();

        lastTimestamp[0] = edu.wpi.first.wpilibj.RobotController.getFPGATime();
        lastTimestamp[1] = edu.wpi.first.wpilibj.RobotController.getFPGATime() - 20; 
    }
    public static void zeroPos(){
        gyroOffset = Math.toRadians(gyroObject.getAngle());
        poseEstimator.resetPose(new Pose2d(0,0,Rotation2d.fromRadians(0)));
    }
    public static void ResetDirection() {
        gyroOffset -= getOffsetGyroRotationRad();
    }
    public static Pose2d getRobotPose() {
        return lastPose[0];
    }
    
    public static Pose2d getPoseTranslated(Pose2d offset){
        return PositionTools.getPoseTranslated(getRobotPose(), offset);
    }
    public static ChassisSpeeds getChassisSpeeds(velType velMethod){
        switch(velMethod){
            case kOdometry:
                Transform2d delta = lastPose[0].minus(lastPose[1]).div((double)((lastTimestamp[0] - lastTimestamp[1])/1000000));
                return new ChassisSpeeds(delta.getX(), delta.getY(), delta.getRotation().getRadians());
            case kGyroscope:
                return new ChassisSpeeds(gyroObject.getVelocityX(), gyroObject.getVelocityY(), gyroObject.getVelocityZ());
            case kAverage:
                break;
            default:
                break;
        }
        return null;
    }
    public static ChassisSpeeds getChassisSpeeds(){
        return getChassisSpeeds(PositionComponentSettings.defaultVelType);
    }

    private static double getGyroRotation(){
        return gyroObject.getRotation2d().getDegrees();
    }

    public static double getOffsetGyroRotationRad(){

        // double r = (currentRad + gyroOffset + Math.PI) / (2 * Math.PI) - Math.PI;
        double r = currentRad + gyroOffset + Math.PI;
        //-2 Pi <> 2 Pi
        r %= 2 * Math.PI;
        //0 <> 4 Pi
        r += 2 * Math.PI;
        //0 <> 2Pi
        r %= 2 * Math.PI;
        //-Pi <> Pi
        r -= Math.PI;
        return r;
    }
    public static double getOffsetGyroRotation() {
        return Math.toDegrees(getOffsetGyroRotationRad());
        //8.774176	4.0259
    }

    public static void updatePose(LimelightComponent.PoseWithTimestamp limelightPos){
        if(limelightPos == null) return;
        if(!limelightPos.noRotation) {
            double delta = limelightPos.pose.getRotation().getRadians() - getOffsetGyroRotationRad();
            // System.out.println("lime " + limelightPos.pose.getRotation().getRadians());
            // System.out.println("gyro " + getOffsetGyroRotationRad());
            // System.out.println("delta " + delta);55
            gyroOffset += delta; 
            gyroOffset %= 2 * Math.PI;
        }
        
        poseEstimator.addVisionMeasurement(new Pose2d(limelightPos.pose.minus(new Pose2d(8.774176,4.0259,Rotation2d.kZero)).getTranslation(), limelightPos.pose.getRotation()), limelightPos.timestamp);
    }

    public static void periodic(){
        currentRad = -gyroObject.getRotation2d().getRadians();
        poseEstimator.update(Rotation2d.fromRadians(getOffsetGyroRotationRad()), Drivetrain.getSwerveModulePositions());
        if(LimelightComponent.active() && LimelightComponent.calcAprilTag() != null) updatePose(LimelightComponent.calcAprilTag());
        lastTimestamp[1] = lastTimestamp[0];
        lastTimestamp[0] = edu.wpi.first.wpilibj.RobotController.getFPGATime();
        lastPose[1] = lastPose[1];
        lastPose[0] = poseEstimator.getEstimatedPosition();

        // if(LimelightComponent.calcAprilTag() != null){
        //     updatePose();
        // }        
    }
}