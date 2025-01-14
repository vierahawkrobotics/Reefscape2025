package frc.robot.Components;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AreaEffect{
    // Simple abstraction so I don't have to create an 8-parameter method
    public class Point{
        public double x;
        public double y;
        public Point(double x, double y){
            this.x = x;
            this.y = y;
        }
    }
    public class TriggerObject{
        public Command cmd;
        public boolean triggered;
        public triggerCondition triggerType;
        public boolean check(Point t){
            return true;
        }
        public void trigger(){
            cmd.schedule();
        }
    }
    public class TriggerObjectPolygon extends TriggerObject{
        private Point[] pointList;

        public TriggerObjectPolygon(Command command, Point[] pointList){
            cmd = command;
            this.pointList = pointList;
        }

        public TriggerObjectPolygon(Command command, double[] xList, double[] yList){
            cmd = command;
            this.pointList = new Point[xList.length];
            for(int i=0;i<xList.length;i++){
                this.pointList[i] = new Point(xList[i], yList[i]);
            }
        }

        // Use slopes to determine whether the points 
        private boolean ccw(Point A, Point B, Point C){
            return ((C.y-A.y)*(B.x-A.x) > (B.y-A.y)*(C.x-A.x));
            
        }
        private boolean vectorIntersects(Point p1, Point p2, Point p3, Point p4){
            
            return ccw(p1,p3,p4) != ccw(p2,p3,p4) && ccw(p1,p2,p3) != ccw(p1,p2,p4);
        }
        private boolean onVector(Point p1, Point p2, Point p3){
            return ((p2.y - p1.y) * (p3.x - p1.x) == (p3.y - p1.y) * (p2.x - p1.x) && p3.x >= Math.min(p1.x,p2.x) && p3.x <= Math.min(p1.x,p2.x));
        }
        @Override
        public boolean check(Point point){
            double x = point.x;
            double y = point.y;

            double minX = pointList[0].x, maxX = pointList[0].x, minY = pointList[0].y, maxY = pointList[0].y;
            for(int i=1; i<pointList.length;i++){
                if(pointList[i].x > maxX) maxX = pointList[i].x;
                if(pointList[i].x < minX) minX = pointList[i].x;
                if(pointList[i].y > maxY) maxY = pointList[i].y;
                if(pointList[i].y < minY) maxY = pointList[i].y;
            }

            if(x > maxX || x < minX || y > maxY || y < minY){
                return false;
            }

            int intersections = 0;
            Point checkPoint = new Point(minX-0.1, minY-0.1);
            for(int i=0;i<pointList.length;i++){
                if(!onVector(point, checkPoint, pointList[i])){
                    intersections += vectorIntersects(pointList[i], pointList[(i+1) % pointList.length], checkPoint, point) ? 1 : 0;
                }
            }
            return intersections % 2 == 1;
        }
    }
    public class TriggerObjectRadius extends TriggerObject{
        private Command cmd;
        private double r;
        private Point center;

        public TriggerObjectRadius(Command command, double r, Point center){
            this.cmd = command;
            this.r = r;
            this.center = center;
        }

        @Override
        public boolean check(Point t){
            return Math.sqrt((t.x - center.x) * (t.x - center.x) + (t.y - center.y) * (t.y - center.y)) <= r;

        }
    }

    private static List<TriggerObject> triggers;
    
    public AreaEffect(TriggerObject trigger){
        triggers.add(trigger);
    }
    public void periodic(){
        Pose2d currentPose = PositionComponent.getRobotPose();
        Point currentPoint = new Point(currentPose.getX(), currentPose.getY());
        TriggerObject currentTrigger;
        for(int i=0;i<triggers.size();i++){
            currentTrigger = triggers.get(i);
            if(currentTrigger.check(currentPoint)){
                if(currentTrigger.triggerType == triggerCondition.onEnter && !currentTrigger.triggered){
                    currentTrigger.trigger();
                } else if(currentTrigger.triggerType == triggerCondition.whileIn){
                    currentTrigger.trigger();
                }
                currentTrigger.triggered = true;
            } else {
                if(currentTrigger.triggerType == triggerCondition.onLeave && currentTrigger.triggered){
                    currentTrigger.trigger();
                } else if(currentTrigger.triggerType == triggerCondition.whileOut){
                    currentTrigger.trigger();
                }
                currentTrigger.triggered = false;
            }
        }
    }

    public enum triggerCondition{
        onEnter,
        onLeave,
        whileIn,
        whileOut
    }
    /**
     * Triggers a command if the robot is in a particular area
     */

    /**
     * return NaN if outside 
     * return target real number if in area
     */
    //float GetTargetRad(float x,float y) 

    //Pose2d getRobotPose()

    //Pose2d return the closest or the coral docking location by an area

    /*
     * List of obj with shape information, access based on which location it is in. Each stores a periodic,
     * on-entry, max speed, and target position variables. 
     */
}
