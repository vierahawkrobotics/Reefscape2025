//----------------------------------------------------------------IMPORTS-----------------------------------------------------------------------------------
package frc.robot.Drivetrain;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Components.AreaEffects.AreaEffectsHandler;
import frc.robot.Components.PositionComponent.PositionComponent;

public class Drivetrain extends SubsystemBase {
//---------------------------------------------------------------VARIABLES-----------------------------------------------------------------------------------
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
//------------------------------------------------------CONSTRUCTOR, SINGLETON, & PERIODIC-----------------------------------------------------------------------------------
    
    private Drivetrain() {}
    /**
    * getInstance method for the drivetrain. Follows the singleton design pattern
    * @author Giahna C.
    * @return The instance of the drivetrain
    */

    public static Drivetrain getInstance(){
        if(instance == null){
            instance = new Drivetrain();
        }
        return instance;
    }
    
    @Override 
    public void periodic(){}
//-----------------------------------------------------------------SETTING PIDS-----------------------------------------------------------------------------------
    /**
    * Sets the PIDs for the drivetrain motor controllers based on velocity. setDrivePositionPIDs automatically calls this method.
    * @author Giahna C.
    * @param vx The velocity for the bot along the x axis according to NWU
    * @param vy The velocity for the bot along the y axis according to NWU
    * @param rot The velocity for the bot along the z axis (rotation)
    * @param DriveUsingNormalizedVectors If vx and vy are in the interval [-1,1], this should likely be true. The program will apply the speed before setting
    * the PIDs. If vx and vy are in meters/ second, set this to false.
    * @param TurnUsingNormalizedVectors If rot is in the interval [-1,1], this should likely be true. The program will apply the rotation speed before settings
    * this PID. If rot is in radians/ second, set this to false.
    * @return The instance of the drivetrain
    */
    public void setVelocityPIDs(double vx, double vy, double rot, boolean DriveUsingNormalizedVectors, boolean TurnUsingNormalizedVectors){
        double speed;
        double rSpeed;
        if (DriveUsingNormalizedVectors){
            updateDriveSpeed();
            speed = driveSpeed;
        } 
        else {
            vx = MathUtil.clamp(vx, -DrivetrainConstants.maxDriveSpeed, DrivetrainConstants.maxDriveSpeed);
            vy = MathUtil.clamp(vy, -DrivetrainConstants.maxDriveSpeed, DrivetrainConstants.maxDriveSpeed);
            speed = 1;
        }

        if (TurnUsingNormalizedVectors) rSpeed = rotSpeed;
        else {
            rot = MathUtil.clamp(rot, -DrivetrainConstants.maxRotSpeed, DrivetrainConstants.maxRotSpeed);
            rSpeed = 1;
        }

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx * speed, vy * speed, rot * rSpeed, PositionComponent.getRobotPose().getRotation());
        SwerveModuleState[] swerveStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        for(int i = 0; i < 4; i++){
            Rotation2d currentAngle = new Rotation2d(maxSwerveModules[i].turningEncoder.getPosition());
            swerveStates[i].optimize(currentAngle);
            // cosine compensation, optional
            // swerveStates[i].speedMetersPerSecond *= swerveStates[i].angle.minus(currentAngle).getCos();
            maxSwerveModules[i].turningPIDController.setReference(swerveStates[i].angle.getRadians(), ControlType.kPosition);
            maxSwerveModules[i].drivingPIDFController.setReference(swerveStates[i].speedMetersPerSecond, ControlType.kVelocity);
        }
    }
    /**
     * Sets the Drivetrain to a specified point by getting velocities from position PIDs. Calls setVelocityPIDs automatically using these velocities
     * 
     * @author Giahna C.
     * @param xSetpoint The x position to set the drivetrain to that's given by a TrapezoidProfile object (uses NWU)
     * @param ySetpoint The y position to set the drivetrain to that's given by a TrapezoidProfile object (uses NWU)
     * @param rotSetpoint The rotation to set the drivetrain to that's given by a TrapezoidProfile object (uses NWU)
     * @return void
     */
    public void setDrivePositionPIDs(TrapezoidProfile.State xSetpoint, TrapezoidProfile.State ySetpoint, TrapezoidProfile.State rotSetpoint){ 
        Pose2d currentPose = PositionComponent.getRobotPose();
        double vx = xPositionPidController.calculate(currentPose.getX(), xSetpoint.position);
        double vy = yPositionPidController.calculate(currentPose.getY(), ySetpoint.position);
        double vr = rotationPidController.calculate(currentPose.getRotation().getRadians(), rotSetpoint.position);

        setVelocityPIDs(vx, vy, vr, false, false);
    }
    /**
     * Uses positional PIDs to get the velocity needed to reach the specified rotation. setVelocityPIDs should be set using this method.
     * @param targetAngle The angle the bot should be at
     * @return The velocity to reach the targetAngle in radians/ sec
     */
    public double getVelocityToSetTargetAngle(double targetAngle){
        targetAngle = MathUtil.angleModulus(targetAngle); //wrap the angle
        return rotationPidController.calculate(PositionComponent.getRobotPose().getRotation().getRadians(), targetAngle);
    }
    /**
     * Sets the PIDs for the driving and turning motor controllers to be in a "hold position." Here, the wheels form an X which makes it harder
     * to move the bot.
     * @author Giahna C.
     */
    public void holdPosition(){
        for (int i=0; i< 4; i++){
            Rotation2d currentAngle = new Rotation2d(maxSwerveModules[i].turningEncoder.getPosition());
            DrivetrainConstants.holdSwerveStates[i].optimize(currentAngle);
            maxSwerveModules[i].turningPIDController.setReference(DrivetrainConstants.holdSwerveStates[i].angle.getRadians(), ControlType.kPosition);
            maxSwerveModules[i].drivingPIDFController.setReference(DrivetrainConstants.holdSwerveStates[i].speedMetersPerSecond, ControlType.kVelocity);
        }
    }

//----------------------------------------------------------SPEED RELATED METHODS-----------------------------------------------------------------------------------
    /**
     * Updataes the driveSpeed variable used for setting the setVelocityPids method when DriveUsingNormalizedVectors is set to true.
     * @author Giahna C.
     */
    public void updateDriveSpeed(){
        if(AreaEffectsHandler.isAreaEffect() == false || AreaEffectsHandler.getMaxSpeed() == null)
            driveSpeed = DrivetrainConstants.defaultDriveSpeed;
        else 
            driveSpeed = MathUtil.clamp(AreaEffectsHandler.getMaxSpeed(), -DrivetrainConstants.maxDriveSpeed, DrivetrainConstants.maxDriveSpeed);
    }
    /**
     * Checks each MaxSwerveModule to see if it's moving or not. If one of them is moving it returns false.
     * @return whether or not the robot's driving motors all have a negligable velocity
     */
    public boolean isRobotStopped(){
        for(int i =0; i<4; i++){
            if(maxSwerveModules[i].drivingEncoder.getVelocity() > DrivetrainConstants.stoppedVelocity) return false;
        }
        return true;
    }
}
