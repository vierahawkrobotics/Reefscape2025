package frc.robot.Match;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Components.ComponentManager;
import frc.robot.Components.ShuffleboardTools;
import frc.robot.Components.PositionComponent.PositionComponent;
import frc.robot.Components.PositionTools.PositionTools;
import frc.robot.Drivetrain.DrivePoseBased;
import frc.robot.Drivetrain.PathPlanner;

public class AutonomousState {
    private static SendableChooser<Command> autoChooser;
    //private static PathPlanner pathPlanner = new PathPlanner();
            
    public static Command getAutoCommand() {
        
        //input using NWU
        //return new DrivePoseBased(PositionTools.getAutoPostFromAlliance());
        Pose2d firstPose = PositionTools.getAutoPostFromAlliance();
        return new SequentialCommandGroup(
            new DrivePoseBased(firstPose),
            new DrivePoseBased(PositionTools.closestScorePoseEntry(false,firstPose)));
        //return new DrivePoseBased(PositionTools.getPoseFromAlliance());
        //return new PathPlannerAuto("New Auto");
    }
    public static void initialize() {
        RobotState.startTime = Timer.getFPGATimestamp();
        //pathPlanner.initialize();
        //PositionComponent.getInstance().InitPose();
        //autoChooser = AutoBuilder.buildAutoChooser();
        //SmartDashboard.putData("Auto Chooser", autoChooser);
    }
    public static void periodic() {}
    public static void exit() {}
}
