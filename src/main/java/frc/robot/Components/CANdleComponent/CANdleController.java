package frc.robot.Components.CANdleComponent;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;

import frc.robot.Components.CANdleComponent.CANdleConstants.RobotStates;

public class CANdleController {
    private static CANdle candle;

    /**
     * Change state
     * @author Andrew S
     */
    public static void setState(RobotStates state) {
        setColor(state);
    }

    /**
     * Change CANdle color
     * @author Andrew S
     */
    public static void setColor(RobotStates animation) {
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

    /**
     * Initialization function
     * @author Andrew S
     * @author WPILib Example Code
     */
    public static void initialize() {
        candle = new CANdle(CANdleConstants.CANdleID);
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.brightnessScalar = CANdleConstants.onScalar;
        candle.configAllSettings(configAll, 100);
        candle.configLEDType(LEDStripType.RGB);
        candle.setLEDs(245, 34, 189);
    }

    public static void periodic() {}
}
