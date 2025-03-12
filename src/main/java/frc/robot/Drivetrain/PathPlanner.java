package frc.robot.Drivetrain;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Components.PositionComponent.PositionComponent;
public class PathPlanner extends Command {
    
    /*
     *Creates a robot config then in a try catch sets the value to something based on the robot settings
     *in the application
     *
     *Configures the robot with the inputs of Supplier<Pose2d> Consumer<Pose2D> Supplier<ChassisSpeeds> 
     *function to drive robot, PPHolonomicDriveController which has the PIDS,robot config, bool to flip 
     *the auto or not, and the drivetrain instance
     *
     *driveRobotRelative uses the set functions in the drivetrain to set velocity 
     *given from chassis speeds supplier and is called from the config 
    */
    public PathPlanner() {
        addRequirements(Robot.instance.drivetrain);
    }

    //Stuff for autobuilder to configure
    Supplier<Pose2d> getPose = () -> PositionComponent.getRobotPose();
    Consumer<Pose2d> resetPose = (Pose2d pose) -> PositionComponent.zeroPos();
    Supplier<ChassisSpeeds> getRobotRelativeSpeeds = () -> PositionComponent.getChassisSpeeds();


    @Override
    public void initialize() {
        RobotConfig config = null;//This is because java dosnt wanna believe anything in a try catch
      try{
        config = RobotConfig.fromGUISettings();//gets config from Pathplanner application
      } catch (Exception e){
        e.printStackTrace();
      }
      AutoBuilder.configure(
        getPose,//Pose supplier
        resetPose,//reset odomotry function
        getRobotRelativeSpeeds,//chassis speed supplier
        (speeds) -> driveRobotRelative(speeds),//Gives speed to Function to drive robot
        new PPHolonomicDriveController(
          new PIDConstants(5.0, 0.0, 0.0),//PIDS
          new PIDConstants(5.0, 0.0, 0.0)
        ),
        config,//Robot config settings in the Pathplanner application
        () ->{//Inverst auto command if alliance is red
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        Robot.instance.drivetrain //Drivetrain instance
      );
    }
    @Override
    public void execute() {}
    @Override
    public void end(boolean interrupted) {}
    @Override
    public boolean isFinished() {
        return false;
    }
    private void driveRobotRelative(ChassisSpeeds speeds){
        Robot.instance.drivetrain.setInputVel(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);//Sets drivetrain x and y velocities in meters per second
        Robot.instance.drivetrain.setInputVelRot(speeds.omegaRadiansPerSecond);//Sets drivetrain rotatiional velocity in radians per second
    }
}
