package frc.robot.Components;

import frc.robot.Components.AreaEffects.AreaEffectsHandler;

public class ComponentManager {
    static CANdleController CANdle;
    public static void Initialize() {

        ComponentExample.Initialize();
        CANdle = new CANdleController();
    }
    public static void Periodic() {
        CANdle.Periodic();
        AreaEffectsHandler.periodic();
    }
}
