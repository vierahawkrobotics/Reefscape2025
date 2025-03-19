package frc.robot.Components;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;


public class PDHManager {
    private static PowerDistribution powerDistribution;
    public static void Initialize() {
        powerDistribution = new PowerDistribution(16, ModuleType.kRev);
    }
    public static void Periodic() {

    }
    public static void EnableSwitchableChannel() {
        powerDistribution.setSwitchableChannel(true);
        System.out.println("enabled switchable channel");
    }
    public static void DisableSwitchableChannel() {
        powerDistribution.setSwitchableChannel(false);
        System.out.println("disabled switchable channel");
    }
}
