package frc.robot.ArmSubsystem;

public class ArmConstants { // All heights in meters
    final public static int elevatorMotorID = -1;
    final public static int elevatorFollowMotorID = -1;
    final public static double p = 1;
    final public static double i = 0;
    final public static double d = 0;
    final public static double gearRadius = 0.02;
    final public static double pidCoefficient = 0;
    final public static double epsilon = 0.02;
    final public static double neutralMotorBias = 0;
    final public static double resetHeightModeBias = -0.05;
    final public static double dropCollectTime = 5; // Seconds

    final public static double armHeight = -1; // Arm bottom distance from ground
    final public static double maxHeight = 100; // Arm max extension length
    final public static double ground = 0; // Arm lowest height (L1)
    final public static double low = 0.81; // L2
    final public static double high = 1.83; // L3
    final public static double algaeOffset = 0.1;
}