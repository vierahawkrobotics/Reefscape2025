package frc.robot.Drivetrain;

public class Vector {
    double x;
    double y;

    public Vector(double x, double y){
        this.x=x;
        this.y=y;
    }
    /**
     * Subtracts two vectors
     * @param v1 
     * @param v2
     * @return Vector made from subtracting vectors
     */
    public Vector subtract(Vector v1, Vector v2){
        double x = v2.x-v1.x;
        double y = v2.y-v1.y;
        return new Vector(x, y);
    }
    /**
     * Normalizes a vector by dividing x and y by magnitude
     * @param v vector to be normalized
     * @return normalized vector
     */
    public Vector normalize(Vector v){
        double x = v.x/getDistance(v);
        double y = v.y/getDistance(v);

        return new Vector(x, y);
    }
    /**
     * Multiplys each vector v component by a c value
     * @param c multiplication value
     * @param v  Vector to be multiplied
     * @return Vector with multiplied compenent 
     */
    public Vector multiplyVector(double c,Vector v){
        return new Vector(c*v.x, c*v.y);
    }
    private double getDistance(Vector v){
        return Math.sqrt(Math.pow(v.x,2) + Math.pow(v.y,2));
    }
}
