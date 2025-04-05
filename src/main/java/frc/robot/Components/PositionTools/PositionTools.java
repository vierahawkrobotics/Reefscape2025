package frc.robot.Components.PositionTools;

import java.util.Arrays;
import java.util.Optional;
import java.util.OptionalInt;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Components.ShuffleboardTools;
import frc.robot.Components.PositionComponent.PositionComponent;

public class PositionTools {
    private PositionTools(){}
    private static Pose2d trueClosestScore(){
        return PositionComponent.getRobotPose().nearest(Arrays.asList(PositionConstants.ScoringLocations.reefEdges));
    }
    public static Pose2d closestScorePoseEntry(boolean isRotated){
        return getPoseTranslated(trueClosestScore(),
        new Pose2d(-PositionConstants.ScoringLocations.backOffset,PositionConstants.ScoringLocations.sideOffset,Rotation2d.fromDegrees(isRotated ? -90: 0)));
    }

    public static Pose2d closestScorePose(boolean isRotated, double limitOffset){
        Pose2d p = trueClosestScore();
        if(!isRotated){
            return getPoseTranslated(p,
            new Pose2d(0, limitOffset, Rotation2d.fromDegrees(0)));
        } else {
            return new Pose2d(p.getTranslation(),p.getRotation().plus(Rotation2d.kCW_90deg)); //TODO: Figure out CW vs VVW
        }
    }

    private static Pose2d trueClosestScore(Pose2d pose){
        return pose.nearest(Arrays.asList(PositionConstants.ScoringLocations.reefEdges));
    }
    public static Pose2d closestScorePoseEntry(boolean isRotated, Pose2d pose){
        return getPoseTranslated(trueClosestScore(pose),
        new Pose2d(-PositionConstants.ScoringLocations.backOffset,PositionConstants.ScoringLocations.sideOffset,Rotation2d.fromDegrees(isRotated ? -90: 0)));
    }
    public static Pose2d closestScorePose(boolean isRotated, double limitOffset, Pose2d pose){
        Pose2d p = trueClosestScore(pose);
        if(!isRotated){
            return getPoseTranslated(p,
            new Pose2d(0, limitOffset, Rotation2d.fromDegrees(0)));
        } else {
            return new Pose2d(p.getTranslation(),p.getRotation().plus(Rotation2d.kCW_90deg)); //TODO: Figure out CW vs VVW
        }
    }

    public static Pose2d getPoseTranslated(Pose2d origin, Pose2d offset){
        double x = offset.getX();
        double y = offset.getY();
        Rotation2d theta = origin.getRotation();
        return new Pose2d(origin.getX() + x*theta.getCos()-y*theta.getSin(), origin.getY() + y*theta.getCos()+x*theta.getSin(), theta.plus(offset.getRotation()));
    }

    public static Pose2d addPose(Pose2d origin, Pose2d offset){
        return new Pose2d(origin.getTranslation().plus(offset.getTranslation()), origin.getRotation().plus(offset.getRotation()));
    }

    public static double poseDist(Pose2d origin, Pose2d destination){
        Transform2d delta = origin.minus(destination);
        return Math.sqrt((delta.getX() * delta.getX()) + (delta.getY() * delta.getY()));
    }

    public static Pose2d getPoseFromAlliance() {
        // Optional<Alliance> ally = DriverStation.getAlliance();
        // OptionalInt loc = DriverStation.getLocation();

        // if(ally.isEmpty() || loc.isEmpty()) return null;
        // int k = loc.getAsInt();
        // k = 2;
        //if(ShuffleboardTools.getStartLocation() == null) return null;
        //int k = ShuffleboardTools.getStartLocation();
        int k = 3;
        switch(k) {
            case 1:
                return new Pose2d(-1.2192 - 0.387,2.111, Rotation2d.fromDegrees(180));
            case 2:
                return new Pose2d(-1.2192 - 0.387, 0, Rotation2d.fromDegrees(180));
            case 3:
                return new Pose2d(-1.2192 - 0.387, -2.111, Rotation2d.fromDegrees(180));
        }
        return null;
    }

    public static Pose2d getAutoPostFromAlliance() {
        /*Optional<Alliance> ally = DriverStation.getAlliance();
        OptionalInt loc = DriverStation.getLocation();

        if(ally.isEmpty() || loc.isEmpty()) return new Pose2d();
        int k = loc.getAsInt();*/
        //if(ShuffleboardTools.getStartLocation() == null) return null;
        //int k = ShuffleboardTools.getStartLocation();
        int k = 3;
        switch(k) {
            case 1:
                return new Pose2d(-2.5,2.111, Rotation2d.fromDegrees(270));
            case 2:
                return new Pose2d(-2.5, 0, Rotation2d.fromDegrees(180));
            case 3:
                return new Pose2d(-2.5, -2.111, Rotation2d.fromDegrees(90));
        }
        return PositionComponent.getRobotPose();
    }
}
