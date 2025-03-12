package frc.robot.Components.CANdleComponent;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;

import frc.robot.Components.CANdleComponent.CANdleConstants.RobotStates;

public class CANdleController {
    private static CANdle candle;

    /**
     * Change state
     */
    public static void setState(RobotStates state) {
        setAnimation(state);
    }

    /**
     * Change CANdle animation
     */
    public static void setAnimation(RobotStates animation) {
        switch (animation) {
            case Intaking:
                candle.setLEDs(255, 23, 0);
                break;
            case Dropping:
                candle.setLEDs(27, 121, 222);
                break;
            case Idle:
            default:
                candle.setLEDs(245, 34, 189);
                break;
        }
    }

    public static void Initialize() {
        candle = new CANdle(CANdleConstants.CANdleID);
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = CANdleConstants.onScalar;
        candle.configAllSettings(configAll, 100);
        candle.configLEDType(LEDStripType.RGB);
        candle.setLEDs(245, 34, 189);
    }

    public static void Periodic() {}
}
