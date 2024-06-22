package frc.robot.Accessibility;

import edu.wpi.first.wpilibj.shuffleboard.*;

public class GUI {

    private static ShuffleboardTab tab;
    public static void initialize() {
        tab = Shuffleboard.getTab("main");
    }

    public static void periodic() {
        tab.add("myValue", 12);
    }
}
