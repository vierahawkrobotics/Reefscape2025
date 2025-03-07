package frc.robot.Drivetrain;
public class Vector {
    double x;
    double y;

    public Vector(double x, double y){
        this.x=x;
        this.y=y;
    }
    public Vector subtract(Vector v2){
        return new Vector(this.x - v2.x, this.y-v2.y);
    }
    public Vector normalize(){
        return new Vector(this.x/getDistance(this), this.y/getDistance(this));
    }
    public double magnitude(){
        return Math.sqrt(Math.pow(x,2) + Math.pow(y,2));
    }
    public double getDistance(Vector v){
        return Math.sqrt(Math.pow(x-v.x,2) + Math.pow(y-v.y,2));
    }
    public Vector multiplyVector(double c,Vector v){
        return new Vector(c*v.x, c*v.y);
    }
}