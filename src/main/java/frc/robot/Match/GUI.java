package frc.robot.match;

import edu.wpi.first.wpilibj.shuffleboard.*;

public class GUI {

    private static ShuffleboardTab tab;
    public static void initialize() {
        tab = Shuffleboard.getTab("main");
        tab.add("myValueGUI", 12);
    }

    public static void periodic() {
    }
}
