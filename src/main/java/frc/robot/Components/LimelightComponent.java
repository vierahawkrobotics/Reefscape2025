package frc.robot.Components;

import frc.robot.LimelightHelpers;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class LimelightComponent {
    public static class PoseWithTimestamp{
        public PoseWithTimestamp(double t, Pose2d p){this.timestamp=t;this.pose=p;}
        public double timestamp;
        public Pose2d pose;
    }

    private static Pose2d lPos = new Pose2d();
    public static Pose2d getLastAprilTag(){
        calcAprilTag();
        return lPos;

    }
    public static PoseWithTimestamp calcAprilTag() {
        LimelightHelpers.PoseEstimate limelightMeasurement = null;
        boolean hasTarget = LimelightHelpers.getTV("");

        Optional<Alliance> ally = DriverStation.getAlliance();
        if(!hasTarget || ally.isEmpty()) return null;

        if (ally.get() == Alliance.Red) {
            limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiRed("");
        } else if(ally.get() == Alliance.Blue){
            limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("");
        }

        if(limelightMeasurement != null){
            lPos = limelightMeasurement.pose;
            return new PoseWithTimestamp(limelightMeasurement.timestampSeconds, limelightMeasurement.pose);
        } else {
            return null;
        }
    }
}
