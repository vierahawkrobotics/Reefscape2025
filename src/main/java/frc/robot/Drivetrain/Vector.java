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
    public double getDistance(Vector v){
        return Math.sqrt(Math.pow(v.x,2) + Math.pow(v.y,2));
    }
    public Vector multiplyVector(double c,Vector v){
        return new Vector(c*v.x, c*v.y);
    }
}