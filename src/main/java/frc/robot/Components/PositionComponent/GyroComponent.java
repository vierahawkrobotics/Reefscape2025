package frc.robot.Components.PositionComponent;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Components.LimelightComponent;
import frc.robot.Components.ShuffleboardTools;
import frc.robot.Components.PositionTools.PositionTools;

public class GyroComponent {
    private AHRS gyro;
    private double gyroOffset = 0;
    private double currentRad = 0;
    private Pose2d initPose = null;
    private Pose2d origin = new Pose2d();
    private Translation2d gyroPos = new Translation2d(-0.1937,0.1334);
    private Translation2d velocity = new Translation2d();
    private Translation2d position = new Translation2d();
    private double time = 0;
    public GyroComponent(){
        gyro = new AHRS(NavXComType.kMXP_SPI);
        time = Timer.getFPGATimestamp();
    }

    public void updatePose(LimelightComponent.PoseWithTimestamp limelightPos){
        if(limelightPos == null) return;
        if(!limelightPos.noRotation) {
            double delta = limelightPos.pose.getRotation().getRadians() - getRotation().getRadians();
            gyroOffset += delta; 
            gyroOffset %= 2 * Math.PI;
        }
        
        origin = new Pose2d(limelightPos.pose.getX() - position.getX(), limelightPos.pose.getY() - position.getY(), limelightPos.pose.getRotation().minus(getRotation()));
        //Tentative offset: new Pose2d(8.774176,4.0259,Rotation2d.kZero)
        //poseEstimator.addVisionMeasurement(limelightPos.pose, limelightPos.timestamp);
    }

    public Translation2d getVelocity() {
        return velocity;
    }

    public Rotation2d getRotation() {
        return Rotation2d.fromRadians(currentRad);
    }

    public void Update() {
        currentRad = toRotation(-gyro.getRotation2d().getRadians() + gyroOffset);

        if(initPose == null && ShuffleboardTools.getStartLocation()!=null) {
            Pose2d p = PositionTools.getPoseFromAlliance();
            if(p != null) {
                initPose = p;
            }
        }

        double x = - gyro.getRobotCentricVelocityY();
        double y = - gyro.getRobotCentricVelocityX();
        velocity = new Translation2d(x,y);

        double ntime = Timer.getFPGATimestamp();
        double dt = ntime - time;

        position.plus(velocity.times(dt / 2));        
        
        time = ntime;
    }

    private static double toRotation(double rad){

        // double r = (currentRad + gyroOffset + Math.PI) / (2 * Math.PI) - Math.PI;
        double r = rad + Math.PI;
        //-2 Pi <> 2 Pi
        r %= 2 * Math.PI;
        //0 <> 4 Pi
        r += 2 * Math.PI;
        //0 <> 2Pi
        r %= 2 * Math.PI;
        //-Pi <> Pi
        r -= Math.PI;
        return r;
    }
}
