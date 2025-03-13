package frc.robot.Match;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drivetrain.DrivePoseBased;

public class AutonomousState {
    public static Command getAutoCommand() {
        //input using NWU
        return new DrivePoseBased(-6,-4,Math.PI/2);
    }
    public static void initialize() {}
    public static void periodic() {}
    public static void exit() {}
}
