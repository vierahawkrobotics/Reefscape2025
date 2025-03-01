package frc.robot.Match;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drivetrain.DrivePoseBased;

public class AutonomousState {
    public static Command getAutoCommand() {
        return new DrivePoseBased(0,1,Math.PI);
    }
    public static void initialize() {}
    public static void periodic() {}
    public static void exit() {}
}
