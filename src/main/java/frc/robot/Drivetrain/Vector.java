package frc.robot.Drivetrain;

public class Vector {
    double x;
    double y;

    public Vector(double x, double y){
        this.x=x;
        this.y=y;
    }
    public Vector subtract(Vector v1, Vector v2){
        double x = v2.x-v1.x;
        double y = v2.y-v1.y;
        return new Vector(x, y);
    }
    public Vector normalize(Vector v){
        double x = v.x/getDistance(v);
        double y = v.y/getDistance(v);

        return new Vector(x, y);
    }
    public double getDistance(Vector v){
        return Math.sqrt(Math.pow(v.x,2) + Math.pow(v.y,2));
    }
    public Vector multiplyVector(double c,Vector v){
        return new Vector(c*v.x, c*v.y);
    }
}
