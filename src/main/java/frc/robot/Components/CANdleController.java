package frc.robot.Components;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
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

                break;
            case Dropping:
                break;
            case AlgaeRemoval:
                break;
            case RobotMoving:
                break;
            case Idle:
            default:
                break;
        }
    }

    public void Periodic() {
        candle.animate(animate);
    }
}
