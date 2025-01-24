package frc.robot.Components;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;

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
        poseEstimator = new SwerveDrivePoseEstimator(null, new Rotation2d(gryoObject.getAngle() * Math.PI / 180), null, initialPose); // Fix kinematics and modulePositions parameter
    }

    public static Pose2d getRobotPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public static void updatePose(){
        poseEstimator.addVisionMeasurement(null, edu.wpi.first.wpilibj.Timer.getFPGATimestamp()); // Fix visionRobotPoseMeters
    }

    public static void perodic(){
        wheelPositions = null; // Fix when drivetrain is complete (ELI)
        poseEstimator.update(new Rotation2d(gryoObject.getAngle() * Math.PI / 180), wheelPositions);
        if(LimelightComponent.calcAprilTag() != null){
            updatePose();
        }        
    }
}