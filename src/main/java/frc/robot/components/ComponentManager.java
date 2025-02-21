package frc.robot.Components;

import frc.robot.Components.AreaEffects.AreaEffectsHandler;

public class ComponentManager {
    public static void Initialize() {
        AreaEffectsHandler.initialize();
        HighwaySystem.initialize();
    }
    public static void Periodic() {
        AreaEffectsHandler.periodic();
    }
}
