package frc.robot.Components.PositionSensors;


public class LimelightSensor{
    private static double mySensorValue = 0;
    public static void Initialize() {
        //initialize camera, sensor, etc.
    }

    public static double getDistance() {
        //return a value from sensor
        return mySensorValue;
    }
}
