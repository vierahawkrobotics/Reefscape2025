package frc.robot.Components;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;

/**
 * @todo: 
 * @author:
 */
public class PositionComponent {
    private static SwerveDrivePoseEstimator poseEstimator;

    public PositionComponent(Pose2d initialPose) {
        poseEstimator = new SwerveDrivePoseEstimator(null, null, null, initialPose);
    }

    public static Pose2d getRobotPose() {
        return poseEstimator.getEstimatedPosition();
    }
    public static void updatePose(){
        poseEstimator.addVisionMeasurement(null, 0);
    }
    public static void perodic(){
        poseEstimator.update();
    }
}
