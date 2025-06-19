package frc.robot.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FollowPath extends SequentialCommandGroup{
    /**
     * Creates a path for the robot to follow composed of GoToPoint objects.
     * @author Giahna C.
     * @param points The array with each point the robot should go to for this path, in order. Each object
     * In this array should be a GoToPoint command.
     */
    public FollowPath(GoToPoint[] points){
        for (GoToPoint point : points){
            addCommands(point);
        }
    }
    /**
     * Creates a path for the robot to follow using double arrays that specify x,y, and rotation.
     * @author Giahna C.
     * @param points A 2D array of the points that compose the path in order. Each point should be: [x, y, rotation] according to NWU.
     */
    public FollowPath(double[][] points){
        for (double[] point : points){
            addCommands(new GoToPoint(point[0],point[1], point[2], true));
        }
    }
    /**
     * Creates a path for the robot to follow using an array of Pose2D objects. 
     * @author Giahna C.
     * @param points An array of Pose2D objects that specify each point that makes up the path in order. Use NWU.
     */
    public FollowPath(Pose2d[] points){
        for (Pose2d point : points){
            addCommands(new GoToPoint(point, true));
        }
    }
    /**
     * Get the point the robot is currently going to.
     * @author Giahna C.
     * @return A Pose2d object with the point. If the method returns null, there is some error where Drivetrain is running another command
     * when it should be running a GoToPoint command.
     */
    public Pose2d getCurrentPoint(){
        Command curCommand = Drivetrain.getInstance().getCurrentCommand();
        if (curCommand instanceof GoToPoint){
            return ((GoToPoint)curCommand).getTargetPosition();
        }
        else return null;
    }
    /**
     * Add a point to the path AFTER the object is made. This will be added to the end of the path.
     * @author Giahna C.
     * @param point A GoToPoint command that should be added to the path.
     */
    public void addPoint(GoToPoint point){
        addCommands(point);
    }
    /**
     * Add a point to the path AFTER the object is made. This will be added to the end of the path.
     * @author Giahna C.
     * @param point A Pose2d with the point that should be added.
     */
    public void addPoint(Pose2d point){
        addCommands(new GoToPoint(point, true));
    }
}
