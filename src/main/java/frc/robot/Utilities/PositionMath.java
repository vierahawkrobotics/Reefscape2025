package frc.robot.Utilities;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * A class for doing math dealing with Pose objects
 */
public class PositionMath {
    /**
     * Gives the linear distance between two Pose2d objects
     * @param a Pose 1
     * @param b Pose 2
     * @return Distance between poses
     */
    public static double distance(Pose2d a, Pose2d b){
        return Math.sqrt((a.getX()-b.getX())*(a.getX()-b.getX())+(a.getY()-b.getY())*(a.getY()-b.getY()));
    }
}
