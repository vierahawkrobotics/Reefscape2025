package frc.robot.Components;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AreaEffect{
    /**
     * A simple class that represents a 2 dimensional point
     * @author Darren Ringer
     */
    public class Point{
        public double x;
        public double y;
        public Point(double x, double y){
            this.x = x;
            this.y = y;
        }
    }

    /**
     * The generic superclass for trigger object types used by area effects
     * Contains a command, trigger type, and trigger state
     * @param cmd The command to be triggered
     * @param triggerType The trigger type enum
     * @author Darren Ringer
     */
    public class TriggerObject{
        public Command cmd;
        public boolean triggered;
        public triggerCondition triggerType;
        /**
         * The generic superclass for trigger object types used by area effects<p>
         * Contains a command, trigger type, and trigger state
         * @param cmd The command to be triggered
         * @param triggerType The trigger type enum
         * @author Darren Ringer
         */
        public TriggerObject(Command command, triggerCondition triggerType){
            this.cmd = command;
            this.triggerType = triggerType;
        }
        public boolean check(Point t){
            return true;
        }
        public void trigger(){
            cmd.schedule();
        }
    }
    /**
     * A trigger object that extends the TriggerObject<p>
     * It uses a polygon to decide whether to trigger the command or not
     */
    public class TriggerObjectPolygon extends TriggerObject{

        private Point[] pointList;
        
        /**
         * A trigger object that extends the TriggerObject<p>
         * It uses a polygon to decide whether to trigger the command or not
         * @param command The command to execute 
         * @param triggerType The trigger condition to execute the command on
         * @param pointList The list of points that define the polygon
         * @author Darren Ringer
         */
        public TriggerObjectPolygon(Command command, triggerCondition triggerType, Point[] pointList){
            super(command, triggerType);
            this.pointList = pointList;
        }

        public TriggerObjectPolygon(Command command, triggerCondition triggerType, double[] xList, double[] yList){
            super(command, triggerType);
            this.pointList = new Point[xList.length];
            for(int i=0;i<xList.length;i++){
                this.pointList[i] = new Point(xList[i], yList[i]);
            }
        }
        
        @Override
        /**
         * A method that checks whether the trigger object's polygon contains a given point implementing
         * the winding number algorithm
         * @param point The point to check if it is interior to (robot pose)
         * @return Boolean as whether to trigger the command
         * @author Darren Ringer
         */
        public boolean check(Point point){
            double x = point.x;
            double y = point.y;

            int windingNumber = 0;
            Point v1, v2;
            double xIntersect;
            for(int i=0;i<pointList.length;i++){
                v1 = pointList[i];
                v2 = pointList[(i+1) % pointList.length];

                if(Math.min(v1.y, v2.y) < y && Math.max(v1.y, v2.y) > y){
                    xIntersect = v1.x + (y-v1.y)*(v1.x-v2.x)/(v1.y-v2.y);
                    if(xIntersect < x){
                        if(v1.y > v2.y) windingNumber++;
                        else windingNumber--;
                    }
                }
            }
            return windingNumber != 0;
        }
    }
    /**
     * The circle brand of trigger objects (now 50% more calcium!1!!11)
     * @author Darren Ringer
     */
    public class TriggerObjectCircle extends TriggerObject{
        private double r;
        private Point center;

        /**
         * The circle brand of trigger objects (now 50% more calcium!1!!11)
         * @param command Command to trigger
         * @param triggerType Triggering mode
         * @param r Radius of circle
         * @param center Center of circle
         */
        public TriggerObjectCircle(Command command, triggerCondition triggerType, double r, Point center){
            super(command, triggerType);
            this.r = r;
            this.center = center;
        }
        /**
         * A method that checks whether a point is interior to the trigger's circle
         * @param point The point to check if it is interior to (robot pose)
         * @return Boolean as whether to trigger the command
         * @author Darren Ringer
         */
        @Override
        public boolean check(Point t){
            return Math.sqrt((t.x - center.x) * (t.x - center.x) + (t.y - center.y) * (t.y - center.y)) <= r;

        }
    }

    // The list of triggers
    private static List<TriggerObject> triggers;
    
    public AreaEffect(TriggerObject trigger){
        triggers.add(trigger); 
    }
    /**
     * Adds a trigger object to the list of area effect triggers
     * @param trigger The trigger object to add
     * @author Darren Ringer
     */
    public void addTrigger(TriggerObject trigger){
        triggers.add(trigger);
    }
    
    /**
     * The periodic method of the area effects<p>
     * Itterates through all the commands, tests if they should be triggered, and schedules them
     * based on their triggering behavior and state
     * @author Darren Ringer
     */
    public void periodic(){
        Pose2d currentPose = PositionComponent.getRobotPose();
        Point currentPoint = new Point(currentPose.getX(), currentPose.getY());
        TriggerObject currentTrigger;
        for(int i=0;i<triggers.size();i++){
            currentTrigger = triggers.get(i);
            if(currentTrigger.check(currentPoint)){
                if(currentTrigger.triggerType == triggerCondition.onEnter && !currentTrigger.triggered){
                    currentTrigger.trigger();

                }
                currentTrigger.triggered = true;
            } else {
                if(currentTrigger.triggerType == triggerCondition.onLeave && currentTrigger.triggered){
                    currentTrigger.trigger();
                }
                currentTrigger.triggered = false;
            }
        }
    }

    public enum triggerCondition{
        onEnter,
        onLeave
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
