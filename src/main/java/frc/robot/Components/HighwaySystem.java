package frc.robot.Components;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.PriorityQueue;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.CubicHermiteSpline;
import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.spline.SplineHelper;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Components.AreaEffects.AreaEffectShapes.Point;
import frc.robot.Components.PositionTools.PositionConstants;
import frc.robot.Drivetrain.Vector;
import edu.wpi.first.math.spline.Spline.ControlVector;

public class HighwaySystem {
    /*
     * Idea for functionality 1: Node-based
     * We have set nodes and edges; when a highway is requested, we 
     */
    private static List<Node> NodeList;
    private static List<Edge> EdgeList;
    private static HighwaySystem instance;

    public static class Node extends Vector{
        public Boolean joinable;
        public double x;
        public double y;
        public List<Edge> outboundEdges;
        public Node(double x, double y, boolean j, List<Edge> outboundEdges){
            super(x,y);
            joinable = j;
            this.outboundEdges = outboundEdges; 
        }
        public Node(double x, double y, List<Edge> outboundEdges){
            super(x,y);
            joinable = true;
            this.outboundEdges = outboundEdges;
        }
        public Node(double x, double y, boolean j){
            super(x,y);
            joinable = j;
            this.outboundEdges = new ArrayList<Edge>();
        }
        public Node(double x, double y){
            super(x,y);
            joinable = true;
            this.outboundEdges = new ArrayList<Edge>();
        }
    }

    public static class Edge{
        public Node start;
        public Node end;
        public double weight;
        public boolean directed;
        /**
         * Tests if another edge intersects this one
         * @param e Other Edge
         * @return If an intersection exists
         */
        public boolean intersects(Edge e){
            double x1 = e.start.x;
            double x2 = e.end.x;
            double x3 = start.x;
            double x4 = end.x;

            double y1 = e.start.y;
            double y2 = e.end.y;
            double y3 = start.y;
            double y4 = end.y;

            if(x1 == x2){
                return x1 > Math.min(x3,x4) && x1 < Math.max(x3,x4); 
            }
            if(x3 == x4){
                return x3 > Math.min(x1,x2) && x3 < Math.max(x1,x2);
            }

            double m1 = (y1 - y2)/(x1 - x2);
            double m2 = (y3 - y4)/(x3 - x4);

            double xIntersect = (((m1 * x1) - (m2 * x3)) + y3 + y1)/(m1 - m2);
            double minX = Collections.min(List.of(x1,x2,x3,x4));
            double maxX = Collections.max(List.of(x1,x2,x3,x4));
            return xIntersect >= minX && xIntersect <= maxX;
        }

        public Edge(Node[] nList, int start, int end){
            this.start = nList[start];
            this.end = nList[end];
            this.weight = this.start.getDistance(this.end);
            this.directed = false;

            this.start.outboundEdges.add(this);
            this.end.outboundEdges.add(this);
        }
        public Edge(Node start, Node end){
            this.start = start;
            this.end = end;
            this.weight = start.getDistance(end);
            this.directed = false;

            this.start.outboundEdges.add(this);
            this.end.outboundEdges.add(this);
        }
        public Edge(Node[] nList, int start, int end, boolean directed){
            this.start = nList[start];
            this.end = nList[end];
            this.weight = this.start.getDistance(this.end);
            this.directed = directed;
            
            this.start.outboundEdges.add(this);
            if(!directed){
                this.end.outboundEdges.add(this);
            }
        }
        public Edge(Node start, Node end, boolean directed){
            this.start = start;
            this.end = end;
            this.weight = start.getDistance(end);
            this.directed = directed;

            this.start.outboundEdges.add(this);
            if(!directed){
                this.end.outboundEdges.add(this);
            }
        }
    }

    private static class ComparableNode extends Node implements Comparable<ComparableNode>{
        public double weight;
        public ComparableNode(Node n){
            super(n.x, n.y, n.joinable, n.outboundEdges);
        }

        @Override
        public int compareTo(ComparableNode o) {
            return this.weight > o.weight ? 1 : -1;
        }
    }
    /**
     * Resets the graph to its default state
     * @author Darren Ringer
     */
    public static void resetAdditionalNodes(){
        NodeList = List.of(PositionConstants.PoseGraphData.borderNodes);
        EdgeList = List.of(PositionConstants.PoseGraphData.borderEdges);
    }

    /**
     * Appends a node to the system network<p>
     * Creates edges excluding any that intersect with border edges
     * @param n Node to add
     */
    public static void addNode(Node n){
        Edge currentEdge;
        boolean validEdge;
        NodeList.add(n);
        for(Node node: PositionConstants.PoseGraphData.borderNodes){
            if(!node.joinable) continue;
            validEdge=true;
            currentEdge = new Edge(node, n);
            for(Edge edge: PositionConstants.PoseGraphData.borderEdges){
                if(edge.intersects(currentEdge)){
                    validEdge = false;
                    break;
                }
            } if(validEdge){
                EdgeList.add(currentEdge);
            }
        }
    }
    /**
     * Calculates the optimal path between two nodes in the graph using 
     * <a href = "https://www.geeksforgeeks.org/introduction-to-dijkstras-shortest-path-algorithm/">Dijkstra's algorithm </a>
     * <p>(assumed reachable, if not, skill issue)</p>
     * @param start Starting node
     * @param end Ending node
     * @return The list of edges to follow
     */
    public static List<Edge> calclulatePath(Node start, Node end){
        /*
         * Iterate through all nodes
         * If all nodes are visited, return
         * 
         * Look at the head of the queue's edges
         * Add all new nodes to the queue
         */
        HashMap<Integer, ComparableNode> nodeMap = new HashMap<Integer, ComparableNode>();
        PriorityQueue<ComparableNode> joe = new PriorityQueue<ComparableNode>();

        if(!NodeList.contains(start)) addNode(start);
        if(!NodeList.contains(end)) addNode(end);
        
        for(int i=0;i<nodeMap.size();i++){
            nodeMap.put(i, new ComparableNode(NodeList.get(i)));
        }

        while(!joe.isEmpty()){

        }
        return null;

    }

    private HighwaySystem(){
        resetAdditionalNodes();
    }
    public static void initialize(){
        instance = new HighwaySystem();
    }
    public static HighwaySystem getInstance(){
        return instance;
    }
    //List of nodes, edges, weights. Determine closest edge to current position, target, return path
    /*public HighwaySystem(){
        var sideStart = new Pose2d(Units.feetToMeters(1.54), Units.feetToMeters(23.23),
            Rotation2d.fromDegrees(-180));
        var crossScale = new Pose2d(Units.feetToMeters(23.7), Units.feetToMeters(6.8),
            Rotation2d.fromDegrees(-160));

        var interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(Units.feetToMeters(14.54), Units.feetToMeters(23.23)));
        interiorWaypoints.add(new Translation2d(Units.feetToMeters(21.04), Units.feetToMeters(18.23)));

        TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(12), Units.feetToMeters(12));
        config.setReversed(true);

        var trajectory = TrajectoryGenerator.generateTrajectory(
            sideStart,
            interiorWaypoints,
            crossScale,
            config);
        State test = trajectory.sample(1);

    }
    public static Pose2d closestScorePose(boolean isRotated){
        
    }
        CubicHermiteSpline joe;
    public static Node calculateClosest(Pose2d position){

    }*/
}
