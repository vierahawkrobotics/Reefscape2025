package frc.robot.Components;

import frc.robot.LimelightHelpers;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class LimelightComponent {
    public static Pose2d calcAprilTag() {
        LimelightHelpers.PoseEstimate limelightMeasurement;
        boolean hasTarget = LimelightHelpers.getTV("");

        if(hasTarget){
            Optional<Alliance> ally = DriverStation.getAlliance();
            if (ally.isPresent()) {
                if (ally.get() == Alliance.Red) {
                    limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiRed("");
                } else if(ally.get() == Alliance.Blue){
                    limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("");
                } else {
                    limelightMeasurement = null;
                }
                return(limelightMeasurement.pose);
            }
        }
        return(null);
    }
}