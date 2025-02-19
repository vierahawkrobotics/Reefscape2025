package frc.robot.Drivetrain;

import java.util.ArrayList;

public class Path {
    private int pathIndex;
    private boolean isPathFinished;
    private double[] currentPoint;
    private double currentPointX;
    private Double currentPointY;
    private double currentPointRot;
    private ArrayList<double[]> arrayOfPoints;
    private int pathLength;
    private double radiusFactor;
    private boolean checkVelocity;

    public void Path(ArrayList<double[]> pointsArray, double radiusFactor, boolean doVelocityCheck){
        pathIndex = 0;
        isPathFinished = false;
        arrayOfPoints = pointsArray;
        this.radiusFactor = radiusFactor;
        checkVelocity = doVelocityCheck;

        pathLength = pointsArray.size();
        currentPoint = pointsArray.get(0);
        currentPointX = currentPoint[0];
        currentPointY = currentPoint[1];
        currentPointRot = currentPoint.length == 3? currentPoint[2]: null;
    }
    public void Path(double radiusFactor, boolean doVelocityCheck){
        pathIndex = 0;
        isPathFinished = false;
        pathLength = 0;
        checkVelocity = doVelocityCheck;

        this.radiusFactor = radiusFactor;
    }

    public double[] getCurrentPoint(){
        return currentPoint;
    }
    public double getCurrentX(){
        return currentPointX;
    }
    public double getCurrentY(){
        return currentPointY;
    }
    public Double getCurrentRot(){
        return currentPointRot;
    }
    public boolean getPathStatus(){
        return isPathFinished;
    }
    public double getRadiusFactor(){
        return radiusFactor;
    }
    public void addPointToPath(double[] point){
        arrayOfPoints.add(point);
        pathLength ++;
    }
    public boolean getVelocityCheckSetting(){
        return checkVelocity;
    }
    public void increaseIndex(){
        pathIndex++;
        if(pathIndex >= pathLength){
            isPathFinished = true;
        }
        currentPoint = arrayOfPoints.get(pathIndex);
        currentPointX = currentPoint[0];
        currentPointY = currentPoint[1];
        currentPointRot = currentPoint[2];
    }
    public int getPathLength(){
        return pathLength;
    }
    public int getPathIndex(){
        return pathIndex;
    }
}
