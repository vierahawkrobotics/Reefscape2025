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
                    return minHeight;
            }
        }
    }
    public enum IntakeState {
        Rest,
        Collect,
        Drop,
    }
    public enum AlgaeMotorState {
        ActiveTemp,
        Active,
        Inactive;

        double getMotorState() {
            switch(this) {
                case Active:
                case ActiveTemp:
                    return algaeMotorSpeed;
                case Inactive:
                default:
                    return 0;
            }
        }
    }
    public enum ContainerLimitSwitchState {
        None,
        FarLeft,
        Left,
        Right,
        FarRight;

        // double getOffset() {
        //     switch(this) {
        //         case FarLeft:
        //             return farLeftIntakeChannel;
        //         case Left:
        //             return middleLeftIntakeChannel;
        //         case Right:
        //             return middleRightIntakeChannel;
        //         case FarRight:
        //             return farRightIntakeChannel;
        //         case None:
        //         default:
        //             return 0;
        //     }
        // }
    }

    // Motors Constants
    final public static int elevatorMotorID = 11;
    final public static int elevatorFollowMotorID = 12;
    final public static int containerMotorID = 13; // Bottom
    final public static int containerFollowMotorID = 14; // Top
    // final public static int algaeMotorID = 15;
    final public static double algaeMotorSpeed = 1;
    final public static double containerMotorSpeedBottomDrop = 0.05;
    final public static double containerMotorSpeedTopDrop = 0.3;
    final public static double containerMotorSpeedBottomCollect = -0.12; //-0.08
    final public static double containerMotorSpeedTopCollect = -0.08; //-0.08
    public static final double encoderPositionFactor = (2 * Math.PI); // Radians
    public static final double encoderVelocityFactor = (2 * Math.PI) / 60.0; // Radians per Second
    public static final int maxAmp = 60;

    // Elevator Constants
    final public static double elevatorP = 3.8;
    final public static double elevatorI = 0;
    final public static double elevatorD = 0;
    final public static double gearRadius = 0.02199;
    final public static double elevatorMotorBias = 0;
    final public static double epsilon = 0.02;
    final public static double resetHeightModeBias = -0.06;
    final public static double containerDropTime = 1; // Seconds
    final public static double algaeEjectTime = 2; // Seconds

    // Arm Constants
    final public static double autoResetHeight = 0.1; // Maximum difference between curHeight and minimum arm height to automatically reset to zero
    final public static double minHeight = 0.7493; // Container bottom distance from ground
    final public static double armForwardOffset = 0.4191; // Arm distance from center of robot
    final public static double maxHeight = 1.121; // Arm max extension length
    final public static double collectHeight = .900; // Collection
    final public static double ground = 0.33; // L1
    final public static double low = 0.79; // L2
    final public static double high = 1.21; // L3
    final public static double algaeOffset = 0.1; // Algae offset from coral
    final public static double coralPipeDistance = .329;

    // Intake Constants
    final public static double farLeftIntakeChannel = .10414;
    final public static double middleLeftIntakeChannel = 0.02286;
    final public static double middleRightIntakeChannel = -0.03302;
    final public static double farRightIntakeChannel = -1.9022;
}