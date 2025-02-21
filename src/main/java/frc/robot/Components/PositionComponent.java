package frc.robot.Components;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Alert.AlertType;
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
    private static PositionComponent instance;

    private PositionComponent(Pose2d initialPose) {
        gryoObject = new AHRS(NavXComType.kMXP_SPI);
        Rotation2d initialRot = Rotation2d.fromDegrees(gryoObject.getAngle());
        poseEstimator = new SwerveDrivePoseEstimator(Drivetrain.kinematics, initialRot, Drivetrain.getSwerveModulePositions(), initialPose); // Fix kinematics and modulePositions parameter
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
    public static Pose2d getRobotPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public static Pose2d getPoseTranslated(Pose2d offset){
        return PositionTools.getPoseTranslated(getRobotPose(), offset);
    }

    public static double getGyroRotation(){
        return gryoObject.getRotation2d().getDegrees();
    }

    public static void updatePose(){
        poseEstimator.addVisionMeasurement(getRobotPose(), edu.wpi.first.wpilibj.Timer.getFPGATimestamp());
    }
    public static void periodic(){
        poseEstimator.update(Rotation2d.fromDegrees(gryoObject.getAngle()), Drivetrain.getSwerveModulePositions());

        // if(LimelightComponent.calcAprilTag() != null){
        //     updatePose();
        // }        
    }
}
