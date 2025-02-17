// package frc.robot.Components;

import edu.wpi.first.math.geometry.Pose2d;

public class LimelightComponent {
    public static Pose2d calcAprilTag() {
        boolean hasTarget = false; // Fix to status
        
        if(hasTarget){
            return new Pose2d(); // Fix to pose2d based on april tag positioning
        }else{
            return null; // No valid target, return null
        }
    }
}