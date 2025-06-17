package frc.robot.Drivetrain;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Components.AreaEffects.AreaEffectsHandler;
import frc.robot.Components.PositionComponent.PositionComponent;
import frc.robot.Drivetrain.MaxSwerveModule;

public class Drivetrain extends SubsystemBase {
    private static Drivetrain instance;
    private SwerveDriveKinematics kinematics = DrivetrainConstants.kinematics;
    private PIDController xPositionPidController = new PIDController(DrivetrainConstants.xPositionP, DrivetrainConstants.xPositionI, DrivetrainConstants.xPositionD);
    private PIDController yPositionPidController = new PIDController(DrivetrainConstants.yPositionP, DrivetrainConstants.yPositionI, DrivetrainConstants.yPositionD);
    private PIDController rotationPidController = new PIDController(DrivetrainConstants.rotationP, DrivetrainConstants.rotationI, DrivetrainConstants.rotationD);
    private MaxSwerveModule[] maxSwerveModules = {
        new MaxSwerveModule(DrivetrainConstants.flDrivingID, DrivetrainConstants.flTurningID),
        new MaxSwerveModule(DrivetrainConstants.frDrivingID, DrivetrainConstants.frTurningID),
        new MaxSwerveModule(DrivetrainConstants.blDrivingID, DrivetrainConstants.blTurningID),
        new MaxSwerveModule(DrivetrainConstants.brDrivingID, DrivetrainConstants.brTurningID)
    };
    private double driveSpeed = DrivetrainConstants.defaultDriveSpeed;
    private double rotSpeed = DrivetrainConstants.defaultRotSpeed;

    private Drivetrain() {}

    public static Drivetrain getInstance(){
        if(instance == null){
            instance = new Drivetrain();
        }
        return instance;
    }
    
    @Override 
    public void periodic(){}

    public void setVelocityPIDs(double vx, double vy, double rot, boolean useNormalizedVectors){
        double speed;
        double rSpeed;
        if (useNormalizedVectors){
            updateSpeed();
            speed = driveSpeed;
            rSpeed = rotSpeed;
        }
        else{
            speed = 1;
            rSpeed = 1;
        }
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(vx * speed, vy * speed, rot * rSpeed);
        SwerveModuleState[] swerveStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        for(int i = 0; i < swerveStates.length; i++){
            Rotation2d currentAngle = new Rotation2d(maxSwerveModules[i].turningEncoder.getPosition());
            swerveStates[i].optimize(currentAngle);
            swerveStates[i].speedMetersPerSecond *= swerveStates[i].angle.minus(currentAngle).getCos();
            maxSwerveModules[i].turningPIDController.setReference(swerveStates[i].angle.getRadians(), ControlType.kPosition);
            maxSwerveModules[i].drivingPIDFController.setReference(swerveStates[i].speedMetersPerSecond, ControlType.kVelocity);
        }
    }

    public void setPositionPIDs(TrapezoidProfile.State xSetpoint, TrapezoidProfile.State ySetpoint, TrapezoidProfile.State rotSetpoint){ 
        Pose2d currentPose = PositionComponent.getRobotPose();
        double vx = xPositionPidController.calculate(currentPose.getX(), xSetpoint.position);
        double vy = yPositionPidController.calculate(currentPose.getY(), ySetpoint.position);
        double vr = rotationPidController.calculate(currentPose.getRotation().getRadians(), rotSetpoint.position);

        setVelocityPIDs(vx, vy, vr, false);
    }
    public void updateSpeed(){
        if(AreaEffectsHandler.isAreaEffect() == false || AreaEffectsHandler.getMaxSpeed() == null)
            driveSpeed = DrivetrainConstants.defaultDriveSpeed;
        else 
            driveSpeed = AreaEffectsHandler.getMaxSpeed();
    }
    public boolean isRobotStopped(){
        for(int i =0; i<4; i++){
            if(maxSwerveModules[i].drivingEncoder.getVelocity() > DrivetrainConstants.stoppedVelocity) return false;
        }
        return true;
    }
}
