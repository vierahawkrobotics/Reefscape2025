package frc.robot.Components.PositionComponent;

import java.util.Optional;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Components.LimelightComponent;
import frc.robot.Components.PositionComponent.PositionComponentSettings.*;
import frc.robot.Components.PositionTools.PositionTools;
import frc.robot.Drivetrain.Drivetrain;

/**
 * @todo: Implement
 * @author:Richard Wright
 */
public class PositionComponent {
    private static SwerveDrivePoseEstimator poseEstimator;
    private static SwerveModulePosition[] wheelPositions;
    private static AHRS gryoObject;
    private static Pose2d[] lastPose = new Pose2d[2];
    private static long[] lastTimestamp = new long[2];
    private static PositionComponent instance;

    private PositionComponent(Pose2d initialPose) {
        gryoObject = new AHRS(NavXComType.kMXP_SPI);
        gryoObject.reset();
        Rotation2d initialRot = Rotation2d.fromDegrees(gryoObject.getAngle());
        poseEstimator = new SwerveDrivePoseEstimator(Drivetrain.kinematics, initialRot, Drivetrain.getSwerveModulePositions(), initialPose); // Fix kinematics and modulePositions parameter
        lastPose[0] = poseEstimator.getEstimatedPosition();
        lastPose[1] = poseEstimator.getEstimatedPosition();

        lastTimestamp[0] = edu.wpi.first.wpilibj.RobotController.getFPGATime();
        lastTimestamp[1] = edu.wpi.first.wpilibj.RobotController.getFPGATime() - 20; 

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
    public static void zeroPos(){
        gryoObject.zeroYaw();
        poseEstimator.resetPose(new Pose2d(0,0,Rotation2d.fromRadians(0)));
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
                return new ChassisSpeeds(gryoObject.getVelocityX(), gryoObject.getVelocityY(), gryoObject.getVelocityZ());
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

    public static double getGyroRotation(){
        return gryoObject.getRotation2d().getDegrees();
    }

    public static void updatePose(Pose2d limelightPos){
        if(PositionTools.poseDist(getRobotPose(),limelightPos) <= PositionComponentSettings.maxLimelightDistance){
            poseEstimator.addVisionMeasurement(limelightPos, edu.wpi.first.wpilibj.Timer.getFPGATimestamp());
        }
    }

    public static void periodic(){
        poseEstimator.update(Rotation2d.fromDegrees(gryoObject.getAngle()), Drivetrain.getSwerveModulePositions());
        if(LimelightComponent.calcAprilTag() != null) updatePose(LimelightComponent.calcAprilTag());
        lastTimestamp[1] = lastTimestamp[0];
        lastTimestamp[0] = edu.wpi.first.wpilibj.RobotController.getFPGATime();
        lastPose[1] = lastPose[1];
        lastPose[0] = poseEstimator.getEstimatedPosition();


        // if(LimelightComponent.calcAprilTag() != null){
        //     updatePose();
        // }        
    }
}
