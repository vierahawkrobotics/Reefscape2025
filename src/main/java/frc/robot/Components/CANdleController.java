package frc.robot.Components;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;

import frc.robot.Components.CANdleConstants.RobotStates;

public class CANdleController {
    private final CANdle candle = new CANdle(CANdleConstants.CANdleID);
    private Animation animate = null;
    
    public CANdleController() {}

    /**
     * Change state
     */
    public void setState(RobotStates state) {
        setAnimation(state);
    }

    /**
     * Change animation
     */
    public void setAnimation(RobotStates animation) {
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
            case RobotMoving:
                animate = new RainbowAnimation(1, 0.1, CANdleConstants.numLED);
            case Idle:
            default:
                animate = null;
        }
    }

    public void Periodic() {
        if (animate == null) {
            candle.configBrightnessScalar(CANdleConstants.offScalar);
        } else {
            candle.configBrightnessScalar(CANdleConstants.onScalar);
            candle.animate(animate);
        }
    }
}
