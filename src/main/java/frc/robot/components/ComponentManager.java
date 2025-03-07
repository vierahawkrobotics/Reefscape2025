package frc.robot.Components;

import frc.robot.Components.AreaEffects.AreaEffectsHandler;
import frc.robot.Components.CANdleComponent.CANdleController;

public class ComponentManager {
    public static void Initialize() {
        CANdleController.Initialize();
        ComponentExample.Initialize();
    }
    public static void Periodic() {
        AreaEffectsHandler.periodic();
    }
}
