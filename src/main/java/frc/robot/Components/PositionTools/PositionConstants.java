package frc.robot.Components.PositionTools;

import edu.wpi.first.math.geometry.Pose2d;

public class PositionConstants {
    /**
     * @todo add scoring locations, field relative meters
     */
    public static class ScoringLocations{
        public static Pose2d topReefEdge = new Pose2d();
        public static Pose2d bottomReefEdge = new Pose2d();
        public static Pose2d topRightReefEdge = new Pose2d();
        public static Pose2d topLeftReefEdge = new Pose2d();
        public static Pose2d bottomRightReefEdge = new Pose2d();
        public static Pose2d bottomLeftReefEdge = new Pose2d();
        public static Pose2d reefEdges[] = {topReefEdge, topRightReefEdge, bottomRightReefEdge, bottomReefEdge, bottomLeftReefEdge, topLeftReefEdge};

        public static double backOffset = 1; // How far back the robot should be from each scoring location (meters)
        public static double limitSwitchOffet = -0.1;
    }
    public static class AprilTagPositions{
        
    }
}
