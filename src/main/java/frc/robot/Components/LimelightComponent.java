package frc.robot.Components;

import frc.robot.LimelightHelpers;
import edu.wpi.first.math.geometry.Pose2d;

public class LimelightComponent {
    public static Pose2d calcAprilTag() {
        boolean hasTarget = LimelightHelpers.getTV("");

        if(hasTarget){
            LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("");
            return(limelightMeasurement.pose); // Could be relative position (Eli thinks that it is absolute)
        }else{
            return(null);
        }
    }
}