package frc.robot.Utilities;

public class MiscMathFunctions {
    /**
     * @author Darren R.
     * @param a
     * @param b
     * @return
     */
    public static double mod(double a, double b){
        return (a%b + b)%b;
    }
    public static double distance(double x1, double x2, double y1, double y2){
        return Math.sqrt(
            Math.pow(x1 - x2, 2) +
            Math.pow(y1 - y2, 2)
        );
    }
}
