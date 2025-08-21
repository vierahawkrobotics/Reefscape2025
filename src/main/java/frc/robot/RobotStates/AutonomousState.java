package frc.robot.RobotStates;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Drivetrain.old.DrivePoseBased;
import frc.robot.Drivetrain.GoToPoint;

public class AutonomousState {
    public static Command getAutoCommand() {
        return new GoToPoint(new Pose2d(1,1,Rotation2d.kZero), true);
    }
    public static void initialize() {}
    public static void periodic() {}
    public static void exit() {}
}
