package frc.robot.Components;

import frc.robot.LimelightHelpers;
import frc.robot.Components.PositionComponent.PositionComponent;
import frc.robot.Components.PositionTools.PositionTools;
import frc.robot.LimelightHelpers.RawFiducial;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    public static final double minDistMT1 = 0.4;    // In meters, the min acceptable distance for an MT1 april tag
    public static final double maxDistMT1 = 0.8;    // In meters, the maximum acceptable distance for an MT1 april tag
    public static final double maxDistMT2 = 10.0;    // In meters, the maximum acceptable distance for an MT2 april tag
    public static final double aprilTagHeight = Units.inchesToMeters(10.5); // In meters, the height of the april tag
    public static final double limelightFOVX = 82;      // In degrees, the horizontal FOV of the limelight
    public static final double limelightFOVY = 56.2;    // In degrees, the vertical FOV of the limelight
    public static final double detectionBuffer = 8;     // In degrees, the angular buffer
    public static final Pose2d limelightOffset = new Pose2d(0, 0, Rotation2d.kZero); // Offset of limelight relative to center of Robot
    public static final Pose2d absoluteOffset = new Pose2d(0,0,Rotation2d.kZero);
    public static final double fieldLength = 17.55; // Distance from Red Alliance to Blue Alliance in meters
    public static final double fieldHeight = 8.05; // Other axis in meters
    private static RawFiducial[] fiducials = null;
    //-------------------------------------------Last Data--------------------------------------------//
    public static double dist;
    private static Pose2d lPos = new Pose2d();
    private static boolean shouldRot = true;
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
        if(fiducials == null || fiducials.length <= 0 || getTX() == null || getTY() == null) return false;
        double tagAngularSize = Units.radiansToDegrees(Math.atan2(aprilTagHeight/2,fiducials[0].distToCamera));
        return !(Math.abs(getTX()) > limelightFOVX/2-tagAngularSize-detectionBuffer || Math.abs(getTY()) > limelightFOVY/2-tagAngularSize-detectionBuffer);
    }
    public static void setSamplingRate(int delay){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("crop").setInteger(delay);
    }

    public static void EnableRotControls(boolean shouldRot) {
        LimelightComponent.shouldRot = shouldRot;
    }
    public static void ToggleRotControls() {
        LimelightComponent.shouldRot ^= true;
    }

    public static PoseWithTimestamp calcAprilTag() {
        fiducials = LimelightHelpers.getRawFiducials("");
        boolean noRotation = false;
        LimelightHelpers.PoseEstimate limelightMeasurement = null;
        Optional<Alliance> ally = DriverStation.getAlliance();
        //double tx = NetworkTableInstance.getDefault().getEntry("").getDoubleArray()[0];
        if(!tagIsValid() || !active() || ally.isEmpty() || (ally.get() != Alliance.Red && ally.get() != Alliance.Blue) || fiducials[0].distToCamera < minDistMT1) return null;

        LimelightHelpers.SetRobotOrientation("", PositionComponent.getOffsetGyroRotation(), 0, 0, 0, 0, 0);

        if(fiducials[0].distToCamera < maxDistMT1){
            limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("");
            noRotation = false;
        }else if(fiducials[0].distToCamera < maxDistMT2){
            limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
            noRotation = true;
        }else{
            limelightMeasurement = null;
        }
        Pose2d offset = Pose2d.kZero;
        if(ally.get() == Alliance.Blue){
            offset = PositionTools.addPose(absoluteOffset, new Pose2d(-fieldLength/2, -fieldHeight/2, Rotation2d.kZero));
        } else {
            offset = PositionTools.addPose(absoluteOffset, new Pose2d(fieldLength/2,fieldHeight/2,Rotation2d.k180deg));
        }
        if(!shouldRot) noRotation = true;
        if(limelightMeasurement != null && limelightMeasurement.rawFiducials != null && limelightMeasurement.rawFiducials.length >= 1){
            lPos = limelightMeasurement.pose;
            dist = limelightMeasurement.rawFiducials[0].distToCamera;
            return new PoseWithTimestamp(limelightMeasurement.timestampSeconds, PositionTools.addPose(PositionTools.getPoseTranslated(limelightMeasurement.pose, limelightOffset.times(-1)), offset),noRotation);
        } else {
            dist = -1;
            return null;
        }
    }
}
