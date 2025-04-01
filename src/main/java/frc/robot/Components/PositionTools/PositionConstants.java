package frc.robot.Components.PositionTools;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Components.HighwaySystem.Edge;
import frc.robot.Components.HighwaySystem.Node;
/**
 * @IMPORTANT ALL COORDINATES SHOULD BE IN FIELD-ABSOLUTE, WITH FIELD CENTER AT (0,0)!!!
 */
public class PositionConstants {
    /**
     * @todo add scoring locations, field relative meters
     */
    /**
0	4.70027	-0.719582
0	5.116322	0
0	4.70027	0.719582
0	3.869182	0.719582
0	3.45313	0
0	3.869182	-0.719582
     */
    public static class ScoringLocations{
        public static Pose2d topRightReefEdge = new Pose2d(-4.36,-0.719582,Rotation2d.fromDegrees(60));
        public static Pose2d topReefEdge = new Pose2d(-4.77,0,Rotation2d.fromDegrees(0));
        public static Pose2d topLeftReefEdge = new Pose2d(-4.36,0.719582,Rotation2d.fromDegrees(300));
        public static Pose2d bottomLeftReefEdge = new Pose2d(-3.51,0.719582,Rotation2d.fromDegrees(240));
        public static Pose2d bottomReefEdge = new Pose2d(-3.11,0,Rotation2d.fromDegrees(180));
        public static Pose2d bottomRightReefEdge = new Pose2d(-3.51,-0.719582,Rotation2d.fromDegrees(120));
        public static Pose2d reefEdges[] = {topRightReefEdge,topReefEdge,topLeftReefEdge,bottomLeftReefEdge,bottomReefEdge,bottomRightReefEdge};

        public static double backOffset = 0.7 + .4 + 0.2; // How far back the robot should be from each scoring location (meters)
    }
    public static class PoseGraphData {
        //TODO figure whatever this is out
        public static Node borderNodes[] = {
            new Node(1,2)
        };
        public static Edge borderEdges[] = {
            new Edge(borderNodes, 0, 0)
        };
    }
}
