package frc.robot.Components;

//use name of class as sensor name/type.
public class ComponentExample {
    private static double mySensorValue = 0;
    public static void Initialize() {
        //initialize camera, sensor, etc.
    }

    public static double getDistance() {
        //return a value from sensor
        return mySensorValue;
    }
}
