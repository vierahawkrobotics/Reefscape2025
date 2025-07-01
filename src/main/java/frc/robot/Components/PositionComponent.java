package frc.robot.Components;

import java.util.function.Supplier;

import com.studica.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Utilities.PositionMath;

/**
 * A singleton class that encapsulates the {@link SwerveDrivePoseEstimator}
 * to give positions from limelight and odometry.
 * @author Darren Ringer
 */
class PositionComponent{
    //-------------------------------------------Constants--------------------------------------------//
    private static final double limelightUncertianty = 1.0;

    
    //------------------------------------------Other stuff-------------------------------------------//


    private static PositionComponent instance;
    private static SwerveDrivePoseEstimator poseEstimator;
    private static Supplier<SwerveModulePosition[]> swerveModulePositionSupplier; 
    private static AHRS gyro;
    private static Pose2d lastCache;

    private PositionComponent(SwerveDriveKinematics kinematics, Supplier<SwerveModulePosition[]> swerveModulePositionsSupplier, Pose2d initialPose){
        gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
        
        //TODO: figure out resetting shenanegains
        // gyro.reset();

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyro.getRotation2d(), swerveModulePositionsSupplier.get(), initialPose);
        lastCache = initialPose;
        this.swerveModulePositionSupplier = swerveModulePositionsSupplier;

    }

    /**
     * Gets the instance of PositionComponent or throws an error if none exists yet
     * @return Current instance of PositionComponent
     */
    public static PositionComponent getInstance(){
        if(instance == null){
            throw new Error("Cannot get instance of PositionComponent before initialization :(");
        }
        return instance;
    }

    /**
     * Initializes the PositionComponent
     * @param kinematics Swerve kinematics
     * @param swerveModulePositions Starting Swerve module positions 
     * @param initialPose Starting Pose
     * @return Instance of PositionComponent
     */
    public static PositionComponent initialize(SwerveDriveKinematics kinematics, Supplier<SwerveModulePosition[]> swerveModulePositions, Pose2d initialPose){
        instance = new PositionComponent(kinematics, swerveModulePositions, initialPose);
        return instance;
    }

    public static void resetPose(Pose2d newPose){
        poseEstimator.resetPose(newPose);
    }

    public static Pose2d getPose2d(){
        return lastCache;
    }

    public static void periodic(){
        poseEstimator.update(gyro.getRotation2d(), swerveModulePositionSupplier.get());
        LimelightComponent.PoseWithTimestamp limelightEstimate = LimelightComponent.calcAprilTag();
        if(limelightEstimate != null && PositionMath.distance(lastCache, limelightEstimate.pose) < limelightUncertianty){
            if(limelightEstimate.megaTag2){
                limelightEstimate.pose = new Pose2d(limelightEstimate.pose.getTranslation(),lastCache.getRotation());
            }
            poseEstimator.addVisionMeasurement(limelightEstimate.pose, limelightEstimate.timestamp);
        }
    }

}