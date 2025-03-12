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
    
    public PathPlanner() {
        addRequirements(Robot.instance.exampleSubsystem);
    }

    Supplier<Pose2d> getPose = () -> PositionComponent.getRobotPose();
    Consumer<Pose2d> resetPose = (Pose2d pose) -> PositionComponent.zeroPos();
    Supplier<ChassisSpeeds> getRobotRelativeSpeeds = () -> PositionComponent.getChassisSpeeds();


    @Override
    public void initialize() {
        RobotConfig config = null;
      try{
        config = RobotConfig.fromGUISettings();
      } catch (Exception e){
        e.printStackTrace();
      }
      AutoBuilder.configure(
        getPose,//Pose supplier
        resetPose,//reset odomotry function
        getRobotRelativeSpeeds,//chassis speed supplier
        (speeds, feedforwards) -> driveRobotRelative(speeds),
        new PPHolonomicDriveController(
          new PIDConstants(5.0, 0.0, 0.0),
          new PIDConstants(5.0, 0.0, 0.0)
        ),
        config,
        () ->{
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        Robot.instance.drivetrain
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
        Robot.instance.drivetrain.setInputVel(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        Robot.instance.drivetrain.setInputVelRot(speeds.omegaRadiansPerSecond);
    }
}
