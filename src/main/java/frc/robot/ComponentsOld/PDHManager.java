package frc.robot.ComponentsOld;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;


public class PDHManager {
    private static PowerDistribution powerDistribution;
    public static void Initialize() {
        powerDistribution = new PowerDistribution(0, ModuleType.kRev);
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
