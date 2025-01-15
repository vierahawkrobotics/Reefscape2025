package frc.robot.Drivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
public class goToCommand extends Command {

    Pose2d targetPos;
    Pose2d currentPos;

    double distance;
    Rotation2d rotation;
    
    public goToCommand(Pose2d targetPos, Pose2d currentPos){
        this.targetPos = targetPos;
        this.currentPos = currentPos;

        double differenceX = targetPos.getX()-currentPos.getX();
        double differenceY = targetPos.getY()-currentPos.getY();

        //Uses pythogreum theorum to find the distance from the current position to the target position
        distance = Math.sqrt(Math.pow(differenceX, 2) + Math.pow(differenceY, 2));
        //Finds the angle of the target Position from the current position
        rotation = Rotation2d.fromDegrees(Math.atan(differenceX/differenceY));
    }

    @Override
    public void initialize() {    }
    @Override
    public void execute() {

    }
    @Override
    public void end(boolean interrupted) {}
    @Override
    public boolean isFinished() {
        return false;
    }
}
