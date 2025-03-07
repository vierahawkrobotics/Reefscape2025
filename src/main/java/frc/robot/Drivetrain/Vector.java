package frc.robot.Drivetrain;
public class Vector {
    double x;
    double y;

    public Vector(double x, double y){
        this.x=x;
        this.y=y;
    }
    public Vector subtract(Vector v2){
        return new Vector(x-v2.x, y-v2.y);
    }
    public Vector normalize(){
        return new Vector(x/magnitude(), y/magnitude());
    }
    public double magnitude(){
        return Math.sqrt(Math.pow(x,2) + Math.pow(y,2));
    }
    public double getDistance(Vector v2){
        return Math.sqrt(Math.pow(x-v2.x,2) + Math.pow(y-v2.y,2));
    }
    public Vector multiplyVector(double c){
        return new Vector(c*x, c*y);
    }
}