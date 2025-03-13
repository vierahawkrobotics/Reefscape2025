package frc.robot.Components;

import frc.robot.LimelightHelpers;
import frc.robot.Components.PositionComponent.PositionComponent;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class LimelightComponent {
    public static final double[] defaultArray = {};
    public static final double minDist = 0;
    public static final double maxDistMT1 = 1.0; // In meters, the maximum acceptable distance for an MT1 april tag
    public static final double maxDistMT2 = 6.0; // In meters, the maximum acceptable distance for an MT1 april tag
    public static double dist;
    public static class PoseWithTimestamp{
        public PoseWithTimestamp(double t, Pose2d p,boolean mt2){this.timestamp=t;this.pose=p;this.megaTag2=mt2;}
        public double timestamp;
        public Pose2d pose;
        public boolean megaTag2;
    }

    private static Pose2d lPos = new Pose2d();
    public static boolean active(){
        return LimelightHelpers.getRawFiducials("").length > 0;
    }
    public static Pose2d getLastAprilTag(){
        //calcAprilTag();
        return lPos;

    }
    public static PoseWithTimestamp calcAprilTag() {
        boolean mt2 = false;
        LimelightHelpers.PoseEstimate limelightMeasurement = null;
        Optional<Alliance> ally = DriverStation.getAlliance();
        //double tx = NetworkTableInstance.getDefault().getEntry("").getDoubleArray()[0];
        if(!active() || ally.isEmpty()) return null;

        if (ally.get() == Alliance.Red) {
            limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiRed("");
            LimelightHelpers.SetRobotOrientation("", PositionComponent.getOffsetGyroRotation() + 180, 0, 0, 0, 0, 0);
                
            if(!(limelightMeasurement.rawFiducials.length > 0)){
                limelightMeasurement = null;
            } else if(limelightMeasurement.rawFiducials[0].distToCamera < maxDistMT1){
                // Continue as normal (MT1)
                mt2 = false;
            }else if(limelightMeasurement.rawFiducials[0].distToCamera < maxDistMT2){
                limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("");
                mt2 = true;
            }else{
                limelightMeasurement = null;
            }
        } else if(ally.get() == Alliance.Blue){
            limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("");
            LimelightHelpers.SetRobotOrientation("", PositionComponent.getOffsetGyroRotation(), 0, 0, 0, 0, 0);
                
            if(!(limelightMeasurement.rawFiducials.length > 0)){
                limelightMeasurement = null;
            } else if(limelightMeasurement.rawFiducials[0].distToCamera < maxDistMT1){
                // Continue as normal (MT1)
                mt2 = false;
            }else if(limelightMeasurement.rawFiducials[0].distToCamera < maxDistMT2){
                limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
                mt2 = true;
            }else{
                limelightMeasurement = null;
            }
        }
        
        if(limelightMeasurement != null){
            lPos = limelightMeasurement.pose;
            dist = limelightMeasurement.rawFiducials[0].distToCamera;
            return new PoseWithTimestamp(limelightMeasurement.timestampSeconds, limelightMeasurement.pose,mt2);
        } else {
            dist = -1;
            return null;
        }
    }
}
