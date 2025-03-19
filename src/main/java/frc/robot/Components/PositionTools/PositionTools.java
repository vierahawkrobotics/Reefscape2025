package frc.robot.Components.PositionTools;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Components.PositionComponent.PositionComponent;

public class PositionTools {
    private PositionTools(){}
    private static Pose2d trueclosestScore(){
        return PositionComponent.getRobotPose().nearest(Arrays.asList(PositionConstants.ScoringLocations.reefEdges));
    }
    public static Pose2d closestScorePoseEntry(boolean isRotated){
        return getPoseTranslated(trueclosestScore(),
        new Pose2d(0,PositionConstants.ScoringLocations.backOffset,Rotation2d.fromDegrees(isRotated ? 90: 0)));
    }

    public static Pose2d closestScorePose(boolean isRotated, double limitOffset){
        if(isRotated){
            return getPoseTranslated(trueclosestScore(),
            new Pose2d(limitOffset, 0, Rotation2d.fromDegrees(0)));
        } else {
            return trueclosestScore().rotateBy(Rotation2d.fromDegrees(90));
        }
    }

    public static Pose2d getPoseTranslated(Pose2d origin, Pose2d offset){
        double x = origin.getX();
        double y = origin.getY();
        Rotation2d theta = origin.getRotation();
        return new Pose2d(x*theta.getCos()-y*theta.getSin(), y*theta.getCos()+x*theta.getSin(), theta.plus(offset.getRotation()));
    }

    public static double poseDist(Pose2d origin, Pose2d destination){
        Transform2d delta = origin.minus(destination);
        return Math.sqrt((delta.getX() * delta.getX()) + (delta.getY() * delta.getY()));
    }
}
