package frc.robot.Match;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Components.ComponentManager;
import frc.robot.Components.PositionComponent.PositionComponent;
import frc.robot.Components.PositionTools.PositionTools;
import frc.robot.Drivetrain.DrivePoseBased;
import frc.robot.Drivetrain.DrivetrainConstants;
import frc.robot.Drivetrain.PathPlanner;

public class AutonomousState {
    private static SendableChooser<Command> autoChooser;
    private static PathPlanner pathPlanner = new PathPlanner();
    static Supplier<Pose2d> getPose = () -> PositionComponent.getRobotPose();
    static Consumer<Pose2d> resetPose = (Pose2d pose) -> PositionComponent.zeroPos();
    static Supplier<ChassisSpeeds> getRobotRelativeSpeeds = () -> PositionComponent.getChassisSpeeds();
            
    public static Command getAutonomousCommand() {
        //input using NWU
        // return new DrivePoseBased(PositionTools.getAutoPostFromAlliance());
        //return new DrivePoseBased(PositionTools.getPoseFromAlliance());
        
        
        
        
        PositionComponent.getInstance().InitPose();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        return new PathPlannerAuto("New Auto");
    }
    public static void initialize() {
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
            new PIDConstants(DrivetrainConstants.drivingP, DrivetrainConstants.drivingI, DrivetrainConstants.drivingD),//PIDS
            new PIDConstants(DrivetrainConstants.turningP, DrivetrainConstants.turningI, DrivetrainConstants.turningD)
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
    public static void periodic() {}
    public static void exit() {}
    private static void driveRobotRelative(ChassisSpeeds speeds){
        Robot.instance.drivetrain.setVelocity(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);//Sets drivetrain x and y velocities in meters per second
        Robot.instance.drivetrain.setVelocityRot(speeds.omegaRadiansPerSecond);//Sets drivetrain rotatiional velocity in radians per second
    }
}
