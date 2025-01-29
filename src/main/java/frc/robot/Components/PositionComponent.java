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
