package frc.robot.Components;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class ShuffleboardTools {
    private static ShuffleboardTab tab;
    private static SendableChooser<Integer> allianceLocation;
    public static void Initialize() {
        tab = Shuffleboard.getTab("Drive tools");
        allianceLocation = new SendableChooser<Integer>();
        allianceLocation.addOption("None", 0);
        allianceLocation.addOption("Far left", 1);
        allianceLocation.addOption("Middle", 2);
        allianceLocation.addOption("Far Right",3);
        tab.add(allianceLocation);

        
    }
    public static void Periodic() {

    }
    public static Integer getStartLocation(){
        return allianceLocation.getSelected();
    }
}
