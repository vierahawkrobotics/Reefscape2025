package frc.robot.testing;

import edu.wpi.first.wpilibj.shuffleboard.*;

public class TestingGUI {

    private static ShuffleboardTab tab;
    public static void initialize() {
        tab = Shuffleboard.getTab("myTab");
    }

    public static void periodic() {
        tab.add("myValue", 12);
    }
}
