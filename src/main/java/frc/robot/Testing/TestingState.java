package frc.robot.Testing;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Drivetrain.ThreeDMotionCommand;

public class TestingState {
    public static void initialize() {  
        ThreeDMotionCommand.getInstance().schedule();
    }
    public static void periodic() {
        CommandScheduler.getInstance().run();
    }
    public static void exit() {
        
    }
}
