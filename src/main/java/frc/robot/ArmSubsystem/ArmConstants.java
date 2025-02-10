package frc.robot.ArmSubsystem;

public class ArmConstants { // All heights in meters
    public enum HeightState {
        CoralHigh,
        CoralLow,
        AlgaeHigh,
        AlgaeLow,
        Collect,
        Ground;

        double getHeight() {
            switch(this) {
                case CoralHigh:
                    return high;
                case CoralLow:
                    return low;
                case AlgaeHigh:
                    return high + algaeOffset;
                case AlgaeLow:
                    return low + algaeOffset;
                case Collect:
                    return collectHeight;
                case Ground:
                default:
                    return ground;
            }
        }
    }
    public enum IntakeState {
        Rest,
        Collect,
        Drop,
    }
    public enum ContainerLimitSwitchState {
        None,
        FarLeft,
        Left,
        Right,
        FarRight;

        double getOffset() {
            switch(this) {
                case FarLeft:
                    return farLeftIntakeChannel;
                case Left:
                    return middleLeftIntakeChannel;
                case Right:
                    return middleRightIntakeChannel;
                case FarRight:
                    return farRightIntakeChannel;
                case None:
                default:
                    return 0;
            }
        }
    }
    public enum CoralDropState {
        L3Left,
        L3Right,
        L2Left,
        L2Right,
        L1;

        HeightState getHeight() {
            switch(this) {
                case L3Left:
                case L3Right:
                    return HeightState.CoralHigh;
                case L2Left:
                case L2Right:
                    return HeightState.CoralLow;
                case L1:
                default:
                    return HeightState.Ground;
            }
        }
    }
    public enum AlgaeMotorState {
        Active,
        Inactive;

        double getMotorState() {
            switch(this) {
                case Active:
                    return algaeMotorSpeed;
                case Inactive:
                default:
                    return 0;
            }
        }
    }
    public enum AlgaeDropState {
        High,
        Low;

        HeightState getHeight() {
            switch(this) {
                case High:
                    return HeightState.AlgaeHigh;
                case Low:
                    return HeightState.AlgaeLow;
                default:
                    return HeightState.Ground;
            }
        }
    }

    final public static int elevatorMotorID = -1;
    final public static int elevatorFollowMotorID = -1;
    final public static int containerMotorID = -1;
    final public static int containerFollowMotorID = -1;
    final public static int algaeMotorID = -1;
    final public static double containerMotorSpeed = 99;
    final public static double algaeMotorSpeed = 99;
    public static final double encoderPositionFactor = (2 * Math.PI); // radians
    public static final double encoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    final public static double elevatorP = 1;
    final public static double elevatorI = 0;
    final public static double elevatorD = 0;
    final public static double gearRadius = 0.02;
    final public static double elevatorMotorBias = 0;
    final public static double epsilon = 0.02;
    final public static double resetHeightModeBias = 0;
    final public static double containerDropTime = 5; // Seconds

    final public static double armHeight = -1; // Arm bottom distance from ground
    final public static double armForwardOffset = 1; // Arm distance from center of robot
    final public static double maxHeight = 100; // Arm max extension length
    final public static double collectHeight = 1; // Height to intake from collecetion area
    final public static double ground = 0; // Arm lowest height (L1)
    final public static double low = 0.81; // L2
    final public static double high = 1.83; // L3
    final public static double algaeOffset = 0.1; // How much higher algae is than coral placement

    final public static double farLeftIntakeChannel = -1;
    final public static double middleLeftIntakeChannel = -0.5;
    final public static double middleRightIntakeChannel = 0.5;
    final public static double farRightIntakeChannel = 1;
}