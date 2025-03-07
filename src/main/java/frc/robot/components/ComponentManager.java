package frc.robot.Components;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Components.AreaEffects.AreaEffectsHandler;

public class ComponentManager {
    private static ShuffleboardTab posTab;
    public static void Initialize() {
        PositionComponent.initialize(new Pose2d());
        AreaEffectsHandler.initialize();
        HighwaySystem.initialize();

        //Shuffleboard stuff
        posTab = Shuffleboard.getTab("Position Data");
        posTab.addDouble("Drive Estimated X", ()->{return PositionComponent.getRobotPose().getX();});
        posTab.addDouble("Drive Estimated Y", ()->{return PositionComponent.getRobotPose().getY();});
        posTab.addDouble("Drive Estimated Rotation (degrees)", ()->{return PositionComponent.getRobotPose().getRotation().getDegrees();});
        posTab.addDouble("Gyroscope Estimated Rotation (degrees)", ()->{return PositionComponent.getGyroRotation();});
        
    }
    public static void Periodic() {
        PositionComponent.periodic();
        AreaEffectsHandler.periodic();
    }
}
