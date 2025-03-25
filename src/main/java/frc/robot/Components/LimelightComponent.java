package frc.robot.Components;

import frc.robot.LimelightHelpers;
import frc.robot.Components.PositionComponent.PositionComponent;
import frc.robot.LimelightHelpers.RawFiducial;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * A class to get positional data from the {@link LimelightHelpers} class with some basic processing
 * @author Darren Ringer
 * @author Richard Wright
 */
public class LimelightComponent {    
    //-------------------------------------------Constants--------------------------------------------//
    public static final double[] defaultArray = {};
    public static final Double[] defaultFiducials = new Double[10];
    public static final double minDist = 0;
    public static final double maxDistMT1 = 1.0;    // In meters, the maximum acceptable distance for an MT1 april ta
    public static final double maxDistMT2 = 6.0;    // In meters, the maximum acceptable distance for an MT1 april tag
    public static final double aprilTagHeight = Units.inchesToMeters(10.5); // In meters, the height of the april tag
    public static final double limelightFOVX = 82;      // In degrees, the horizontal FOV of the limelight
    public static final double limelightFOVY = 56.2;    // In degrees, the vertical FOV of the limelight
    public static final double detectionBuffer = 0;     // In degrees, the angular buffer
        
    //-------------------------------------------Last Data--------------------------------------------//
    public static double dist;
    private static Pose2d lPos = new Pose2d();
    public static class PoseWithTimestamp{
        public PoseWithTimestamp(double t, Pose2d p,boolean mt2){this.timestamp=t;this.pose=p;this.noRotation=mt2;}
        public double timestamp;
        public Pose2d pose;
        public boolean noRotation;
    }
    public static boolean active(){
        return LimelightHelpers.getRawFiducials("").length > 0;
    }
    public static Pose2d getLastAprilTag(){
        //calcAprilTag();
        return lPos;
    }

    //----------------------------------------Helper functions----------------------------------------//
    public static Double getTX(){
        return LimelightHelpers.getTX("");
    }
    public static Double getTY(){
        return LimelightHelpers.getTY("");
    }
    public static boolean tagIsValid(){
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
        if(fiducials == null || fiducials.length <= 0 || getTX() == null || getTY() == null) return false;
        double tagAngularSize = Units.radiansToDegrees(Math.atan2(aprilTagHeight/2,fiducials[0].distToCamera));
        return !(Math.abs(getTX()) > limelightFOVX/2-tagAngularSize-detectionBuffer || Math.abs(getTY()) > limelightFOVY/2-tagAngularSize-detectionBuffer);
    }


    public static PoseWithTimestamp calcAprilTag() {
        boolean noRotation = false;
        LimelightHelpers.PoseEstimate limelightMeasurement = null;
        Optional<Alliance> ally = DriverStation.getAlliance();
        //double tx = NetworkTableInstance.getDefault().getEntry("").getDoubleArray()[0];
        if(!tagIsValid() || !active() || ally.isEmpty()) return null;

        if (ally.get() == Alliance.Red) {
            limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiRed("");
            LimelightHelpers.SetRobotOrientation("", PositionComponent.getOffsetGyroRotation() + 180, 0, 0, 0, 0, 0);
                
            if(limelightMeasurement.rawFiducials == null || limelightMeasurement.rawFiducials.length <= 0) return null;
            if(limelightMeasurement.rawFiducials[0].distToCamera < maxDistMT1){
                // Continue as normal (MT1)
                noRotation = false;
            }else if(limelightMeasurement.rawFiducials[0].distToCamera < maxDistMT2){
                limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("");
                noRotation = true;
            }else{
                limelightMeasurement = null;
            }
        } else if(ally.get() == Alliance.Blue){
            limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("");
            LimelightHelpers.SetRobotOrientation("", PositionComponent.getOffsetGyroRotation(), 0, 0, 0, 0, 0);
                
            if(limelightMeasurement.rawFiducials == null || limelightMeasurement.rawFiducials.length <= 0) return null;
            if(limelightMeasurement.rawFiducials[0].distToCamera < maxDistMT1){
                // Continue as normal (MT1)
                noRotation = false;
            }else if(limelightMeasurement.rawFiducials[0].distToCamera < maxDistMT2){
                limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
                noRotation = true;
            }else{
                limelightMeasurement = null;
            }
        }
        
        if(limelightMeasurement != null && limelightMeasurement.rawFiducials != null && limelightMeasurement.rawFiducials.length >= 1){
            lPos = limelightMeasurement.pose;
            dist = limelightMeasurement.rawFiducials[0].distToCamera;
            return new PoseWithTimestamp(limelightMeasurement.timestampSeconds, limelightMeasurement.pose,noRotation);
        } else {
            dist = -1;
            return null;
        }
    }
}
