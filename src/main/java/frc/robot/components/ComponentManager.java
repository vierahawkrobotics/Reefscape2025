package frc.robot.Components;

import frc.robot.Components.AreaEffects.AreaEffectsHandler;
import frc.robot.Components.CANdle.CANdleController;

public class ComponentManager {
    static CANdleController CANdle;
    public static void Initialize() {

        ComponentExample.Initialize();
        CANdle = new CANdleController();
        CANdleController.Initialize();
    }
    public static void Periodic() {
        CANdleController.Periodic();
        AreaEffectsHandler.periodic();
    }
}
