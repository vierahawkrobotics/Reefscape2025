package frc.robot.Drivetrain;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Components.PositionComponent.PositionComponent;

public class Drivetrain extends SubsystemBase{

//-------------------------------------------Initilization------------------------------------
    //kinematics
    //this should ALWAYS be front left, front right, back left, and then back right
    public static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        DrivetrainConstants.frontLeftLocation, 
        DrivetrainConstants.frontRightLocation, 
        DrivetrainConstants.backLeftLocation, 
        DrivetrainConstants.backRightLocation
    );
    //physical parts
    public static MAXSwerveModule[] maxSwerveModules = {
        new MAXSwerveModule(DrivetrainConstants.flDrivingID,DrivetrainConstants.flTurningID,DrivetrainConstants.flChassisAngularOffset),
        new MAXSwerveModule(DrivetrainConstants.frDrivingID,DrivetrainConstants.frTurningID,DrivetrainConstants.frChassisAngularOffset),
        new MAXSwerveModule(DrivetrainConstants.blDrivingID,DrivetrainConstants.blTurningID,DrivetrainConstants.blChassisAngularOffset),
        new MAXSwerveModule(DrivetrainConstants.brDrivingID,DrivetrainConstants.brTurningID,DrivetrainConstants.brChassisAngularOffset)
      };
    /*states for translation and rotation
    roatation state path doesn't do anything 
    but not have it as vel/pos mode*/
    enum TranslateState{
        velocity,
        position,
        path
    }
    enum RotationState{
        velocity,
        position,
        path
    }
    TranslateState translateState = TranslateState.velocity;
    RotationState rotationState = RotationState.velocity;
    //velocites and positions for driveVelocity and drivePosition
    private double velX =0; //value -1 to 1
    private double velY =0; //value -1 to 1
    private double velR =0; //value -1 to 1

    private double posX = 0; //meters
    private double posY =0; //meters
    private double posR =0; //radians
    //distance for pos based control
    private double distance = 0;
    //velocities for applyDrivetrain
    private double appliedX = 0;
    private double appliedY = 0;
    private double appliedR = 0;
    //velocities for shuffleBoard
    private double velTX;
    private double velTY;
    private double velTR;

    ShuffleboardTab drivetrainTab = Shuffleboard.getTab("Drivetrain");
    Drivetrain(){
        drivetrainTab.addDouble("Robot velR", () -> {return velTR;});
        drivetrainTab.addDouble("Robot velX", () -> {return velTX;});
        drivetrainTab.addDouble("Robot velY", () -> {return velTY;});

        drivetrainTab.addDouble("Robot posR", () -> {return posR;});
        drivetrainTab.addDouble("Robot posX", () -> {return posX;});
        drivetrainTab.addDouble("Robot posY", () -> {return posY;});
    }
//-------------------------------------------Periodic------------------------------------  
    @Override 
    public void periodic(){
        switch(translateState){
            case path:
                //TODO: drivePath();
                break;
            case velocity:
                driveVelocity();
                break;
            case position:
                drivePosition();
                break;
        }
        switch(rotationState){
            case velocity:
                driveVelocityRot();
                break;
            case position:
                drivePositionRot();
                break;
            case path:
                //this should NOT be used
                break;
        }
        applyDrivetrain();
        //set values for shuffleboard
        velTX = velX;
        velTY = velY;
        velTR = velR;
    }
//-------------------------------------------Drive Functions------------------------------------
    private void driveVelocity(){
        setDrivetrain(velX, velY);
        velX = 0;
        velY = 0;
    }
    private void driveVelocityRot(){
        setDrivetrainRot(velR);
        velR = 0;
    }
    private void drivePosition(){
        updateDistance();
        Pose2d currentRobotPosition = PositionComponent.getRobotPose();
        //robot current position
        Vector R = new Vector(currentRobotPosition.getX(), currentRobotPosition.getY());
        //target position
        Vector T = new Vector(posX, posY);
        //normal vector
        Vector V = (T.subtract(R)).normalize();
        double scaleFactor = distance>DrivetrainConstants.pointTolerance? 1: distance/DrivetrainConstants.divNumber;
        //for testing
        scaleFactor = 1;
        setDrivetrain(V.x*scaleFactor, V.y*scaleFactor);
    }
    private void drivePositionRot(){
        double currentAngle = PositionComponent.getRobotPose().getRotation().getRadians();
        double d = (posR - currentAngle)%(2*Math.PI) + 2*Math.PI;
        d = d > Math.PI? d - 2*Math.PI: d;

        double vr = d>DrivetrainConstants.rotTolerance? 1: d/DrivetrainConstants.divNumberRot;
        setDrivetrainRot(vr);
    }
//-------------------------------------Set Drivetrain based on Drive Functions-----------------------------

    //this allows translation and rotation to be seperated
    private void setDrivetrain(double vx, double vy){
        appliedX = vx*DrivetrainConstants.maxSpeed;
        appliedY = vy*DrivetrainConstants.maxSpeed;
    }
    private void setDrivetrainRot(double vr){
        appliedR = vr*DrivetrainConstants.maxRotSpeed;
    }
//-------------------------------------------Apply Set Values------------------------------------
    private void applyDrivetrain(){
        Rotation2d currentRotation = PositionComponent.getRobotPose().getRotation().times(-1);
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(appliedX, appliedY, appliedR, currentRotation);
        SwerveModuleState[] moduleStates = OldDrivetrain.kinematics.toSwerveModuleStates(speeds);
        
        for(int i =0; i<4; i++){
        Rotation2d currentAngle = new Rotation2d(maxSwerveModules[i].turningEncoder.getPosition());
        moduleStates[i].optimize(currentAngle);
        moduleStates[i].speedMetersPerSecond *= moduleStates[i].angle.minus(currentAngle).getCos();
        }

         for(int i = 0; i< 4; i++){
            maxSwerveModules[i].turningPIDController.setReference(moduleStates[i].angle.getRadians(), ControlType.kPosition);
            maxSwerveModules[i].drivingPIDController.setReference(moduleStates[i].speedMetersPerSecond, ControlType.kVelocity);
        }
    }
//------------------------------------------Setter Methods------------------------------------
    public void setInputVel(double vx, double vy){
        if (vx > 1) vx = 1;
        else if (vx < -1) vx = -1;
        velX = vx;
        if (vy > 1) vy = 1;
        else if (vy < -1) vy = -1;
        velY = vy;
        translateState = TranslateState.velocity;
    }
    public void setInputVelRot(double vr){
        if (vr > 1) vr = 1;
        else if (vr < -1) vr = -1;
        velR = vr;
        rotationState = RotationState.velocity;
    }
    public void setTargetPos(double px, double py){
        posX = px;
        posY = py;
        translateState = TranslateState.position;
    }
    public void setTargetPosRot(double pr){
        posR = pr;
        rotationState = RotationState.position;
    }
//------------------------------------------Getter Methods------------------------------------
    public boolean getIsPointReached(){
        return Robot.instance.drivetrain.distance < DrivetrainConstants.validRange;
    }
//-------------------------------------------Misc Methods------------------------------------
    private void updateDistance(){
        //TODO: error messaging to check if this works
        Pose2d currentRobotPosition = PositionComponent.getRobotPose();
        distance = 
        Math.sqrt(
        Math.pow(currentRobotPosition.getX() - posX,2) +
        Math.pow(currentRobotPosition.getY() - posY, 2));
    }
    public boolean checkIsRobotStopped(){
        for(int i =0; i<4; i++){
          if(maxSwerveModules[i].drivingEncoder.getVelocity() > DrivetrainConstants.stoppedVelocity) return false;
        }
        return true;
    }
}