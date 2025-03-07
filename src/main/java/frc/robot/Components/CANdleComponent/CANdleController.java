package frc.robot.Components.CANdleComponent;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;

import frc.robot.Components.CANdleComponent.CANdleConstants.RobotStates;

public class CANdleController {
    private static CANdle candle;
    private static Animation animate;

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
                animate = new ColorFlowAnimation(128, 20, 70, 0, 0.7, CANdleConstants.numLED, ColorFlowAnimation.Direction.Forward);
                break;
            case Dropping:
                animate = new FireAnimation(0.5, 0.7, CANdleConstants.numLED, 0.7, 0.5);
                break;
            case AlgaeRemoval:
                animate = new LarsonAnimation(0, 255, 46, 0, 1, CANdleConstants.numLED, LarsonAnimation.BounceMode.Front, 3);
                break;
            case ElevatorMoving:
                animate = new RainbowAnimation(1, 0.1, CANdleConstants.numLED);
                break;
            case RobotMoving:
                animate = new RgbFadeAnimation(0.7, 0.4, CANdleConstants.numLED);
                break;
            case Climbing:
                animate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, CANdleConstants.numLED);
                break;
            case Idle:
            default:
                animate = null;
                break;
        }
    }

    public static void Initialize() {
        candle = new CANdle(CANdleConstants.CANdleID);
        animate = null;
    }

    public static void Periodic() {
        if (animate == null) {
            candle.configBrightnessScalar(CANdleConstants.offScalar);
        } else {
            candle.configBrightnessScalar(CANdleConstants.onScalar);
            candle.animate(animate);
        }
    }
}
