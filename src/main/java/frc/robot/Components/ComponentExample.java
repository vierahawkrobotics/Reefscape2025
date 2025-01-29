package frc.robot.Components;

//use name of class as sensor name/type.
public class ComponentExample {
    private double mySensorValue = 0;
    public void Initialize() {
        //initialize camera, sensor, etc.
    }

    public double getDistance() {
        //return a value from sensor
        return mySensorValue;
    }
}
