package frc.robot.ArmSubsystem;

public class ArmConstants { // All heights in meters
    public enum HeightState {
        Coral,
        Ground;

        double getHeight() {
            switch(this) {
                case Coral:
                    return low;
                case Ground:
                default:
                    return ground;
            }
        }
    }
    public enum IntakeState {
        Rest,
        Drop;
    }

    // Motors Constants
    final public static int elevatorMotorID = 11;
    final public static int elevatorFollowMotorID = 12;
    final public static int containerMotorID = 13; // Bottom
    final public static int containerFollowMotorID = 14; // Top
    final public static double containerMotorSpeedBottomDrop = 0.3;
    final public static double containerMotorSpeedTopDrop = 0.3;
    public static final double encoderPositionFactor = (2 * Math.PI); // Radians
    public static final double encoderVelocityFactor = (2 * Math.PI) / 60.0; // Radians per Second
    public static final int maxAmp = 60;

    // Elevator Constants
    final public static double elevatorP = 3.8;
    final public static double elevatorI = 0;
    final public static double elevatorD = 0;
    final public static double gearRadius = 0.02199; //Changed?
    final public static double elevatorMotorBias = 0;
    final public static double epsilon = 0.02;
    final public static double resetHeightModeBias = -0.06;
    final public static double containerDropTime = 1; // Seconds
    final public static double algaeEjectTime = 2; // Seconds

    // Arm Constants
    final public static double autoResetHeight = 0.1; // Max difference between curHeight and minHeight to auto reset to 0
    final public static double minHeight = 999999999; // Container bottom distance from floor
    final public static double armForwardOffset = 999999999; // Arm distance from center of robot
    final public static double maxHeight = 999999999; // Arm max extension length from floor
    final public static double ground = 0.33; // L1
    final public static double low = 0.81; // L2
}