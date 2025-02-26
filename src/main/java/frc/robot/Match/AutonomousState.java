package frc.robot.Match;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class AutonomousState {
      private final SendableChooser<Command> autoChooser;


    public static Command getAutoCommand() {
        return new PathPlannerAuto("Auto Name");
    }
    public static void initialize() {
        autoChooser = AutoBuilder.buildAutoChooser();
    }
    public static void periodic() {}
    public static void exit() {}
}
