package frc.robot.Components.PositionComponent;

public class PositionComponentSettings {
    public enum rotType{
        kOdometry,
        kGyroscope
    }
    public enum velType{
        kOdometry,
        kGyroscope,
        kAverage
    }
    public static final velType defaultVelType = velType.kOdometry;
    public static final double maxLimelightDistance = 1;
}
