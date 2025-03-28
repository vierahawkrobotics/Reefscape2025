package frc.robot.Match;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Robot;
import frc.robot.Components.LimelightComponent;
import frc.robot.Components.PDHManager;
import frc.robot.Drivetrain.Drive3D;

public class DisabledState {
    public static void Initialize() {
        PDHManager.DisableSwitchableChannel();
        LimelightComponent.setSamplingRate(200);
    }
    public static void Periodic() {

    }
    public static void Exit() {
        LimelightComponent.setSamplingRate(0);
        PDHManager.EnableSwitchableChannel();
    }
}
