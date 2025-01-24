import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.spline.CubicHermiteSpline;
import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.spline.SplineHelper;
import edu.wpi.first.math.spline.Spline.ControlVector;

public class HighwaySystem {
    public class Node{
        public Pose2d pos;
    }
    public class Edge{
        public Node start;
        public Node end;
        public double weight;
    }
    //List of nodes, edges, weights. Determine closest edge to current position, target, return path
    public HighwaySystem(){
        CubicHermiteSpline joe;
    }
}
