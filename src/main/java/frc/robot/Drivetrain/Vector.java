package frc.robot.Drivetrain;
public class Vector {
    double x;
    double y;

    public Vector(double x, double y){
        this.x=x;
        this.y=y;
    }
    public Vector subtract(Vector v2){
        x = x-v2.x;
        y = y-v2.y;
        return new Vector(x, y);
    }
    public Vector normalize(){
        x = x/getDistance(this);
        y = y/getDistance(this);

        return new Vector(x, y);
    }
    public double getDistance(Vector v){
        return Math.sqrt(Math.pow(v.x,2) + Math.pow(v.y,2));
    }
    public Vector multiplyVector(double c,Vector v){
        return new Vector(c*v.x, c*v.y);
    }
}