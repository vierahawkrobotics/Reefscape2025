package frc.robot.Components;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Drivetrain.Drivetrain;

/**
 * @todo: Implement
 * @author:Richard Wright
 */
public class PositionComponent {
    private static SwerveDrivePoseEstimator poseEstimator;
    private static SwerveModulePosition[] wheelPositions;
    private static AHRS gryoObject;

    public PositionComponent(Pose2d initialPose) {
        gryoObject = new AHRS(SerialPort.Port.kMXP);
        poseEstimator = new SwerveDrivePoseEstimator(Drivetrain.kinematics, Rotation2d.fromDegrees(gryoObject.getAngle()), Drivetrain.getSwerveModulePositions(), initialPose);
    }

    public static Pose2d getRobotPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public static Pose2d getPoseTranslated(Pose2d offset){
        double x = getRobotPose().getX();
        double y = getRobotPose().getY();
        Rotation2d theta = getRobotPose().getRotation();
        return new Pose2d(x*theta.getCos()-y*theta.getSin(), y*theta.getCos()+x*theta.getSin(), theta.plus(offset.getRotation()));
    }

    public static void updatePose(){
        poseEstimator.addVisionMeasurement(getRobotPose(), edu.wpi.first.wpilibj.Timer.getFPGATimestamp());
    }
    public static void perodic(){
        poseEstimator.update(Rotation2d.fromDegrees(gryoObject.getAngle()), Drivetrain.getSwerveModulePositions());

        if(LimelightComponent.calcAprilTag() != null){
            updatePose();
        }        
    }
}
