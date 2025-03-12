package frc.robot.Match;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drivetrain.DrivePoseBased;
import frc.robot.Drivetrain.PathPlanner;

public class AutonomousState {
    private static SendableChooser<Command> autoChooser;
    private static PathPlanner pathPlanner = new PathPlanner();
            
    public static Command getAutoCommand() {
        
        //input using NWU
        return new DrivePoseBased(1,0,Math.PI/2);
    }
    public static void initialize() {
        pathPlanner.initialize();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }
    public static void periodic() {}
    public static void exit() {}
    public Command getAutonomousCommand(){
        return new PathPlannerAuto("New Auto");//Gets auto based on auto name, more info on getting autos here https://pathplanner.dev/pplib-build-an-auto.html#create-a-sendablechooser-with-certain-autos-in-project
    }
}
