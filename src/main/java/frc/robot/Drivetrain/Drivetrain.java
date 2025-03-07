package frc.robot.Drivetrain;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Components.AreaEffects.AreaEffectsHandler;
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
        // new MAXSwerveModule(DrivetrainConstants.flDrivingID,DrivetrainConstants
        // .flTurningID,DrivetrainConstants.flChassisAngularOffset),
        // new MAXSwerveModule(DrivetrainConstants.frDrivingID,DrivetrainConstants
        // .frTurningID,DrivetrainConstants.frChassisAngularOffset),
        // new MAXSwerveModule(DrivetrainConstants.blDrivingID,DrivetrainConstants
        // .blTurningID,DrivetrainConstants.blChassisAngularOffset),
        // new MAXSwerveModule(DrivetrainConstants.brDrivingID,DrivetrainConstants
        // .brTurningID,DrivetrainConstants.brChassisAngularOffset)
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
    private double rotDistance;
    //velocities for applyDrivetrain
    private double appliedX = 0;
    private double appliedY = 0;
    private double appliedR = 0;
    //velocities and other vars for shuffleBoard
    private double velTX;
    private double velTY;
    private double velTR;
    private double setVelX;
    private double setVelY;
    private double VxSB;
    private double VySB;
    private double distanceShuffle = 0;

    ShuffleboardTab drivetrainTab = Shuffleboard.getTab("Drivetrain");
    public Drivetrain(){
        // drivetrainTab.addDouble("Robot velR", () -> {return velTR;});
        // drivetrainTab.addDouble("Robot velX", () -> {return velTX;});
        // drivetrainTab.addDouble("Robot velY", () -> {return velTY;});

        drivetrainTab.addDouble("Robot posR", () -> {return posR;});
        drivetrainTab.addDouble("Robot posX", () -> {return posX;});
        drivetrainTab.addDouble("Robot posY", () -> {return posY;});

        // drivetrainTab.addDouble("Set X Speed", () -> {return setVelX;});
        // drivetrainTab.addDouble("Set Y Speed", () -> {return setVelY;});
        drivetrainTab.addDouble("V.x", () -> {return VxSB;});
        drivetrainTab.addDouble("V.y", () -> {return VySB;});
        drivetrainTab.addDouble("Distance From Point", () -> {return distanceShuffle;});
        drivetrainTab.addString("tran state", () -> {return translateState.toString();});
    }
//-------------------------------------------Periodic------------------------------------  
    @Override 
    public void periodic(){
        // switch(translateState){
        //     case path:
        //         //TODO: drivePath();
        //         break;
        //     case velocity:
        //         driveVelocity();
        //         break;
        //     case position:
        //         drivePosition();
        //         break;
        // }
        // switch(rotationState){
        //     case velocity:
        //         driveVelocityRot();
        //         break;
        //     case position:
        //         drivePositionRot();
        //         break;
        //     case path:
        //         //this should NOT be used
        //         break;
        // }
        // applyDrivetrain();
        // //set values for shuffleboard
        // velTX = velX;
        // velTY = velY;
        // velTR = velR;
    }
//-------------------------------------------Drive Functions------------------------------------
    private void driveVelocity(){
        updateDistance();
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
        double scaleFactor;
        if (distance <= DrivetrainConstants.validRange) scaleFactor = 0;
        else{
            scaleFactor = distance>DrivetrainConstants.pointTolerance? 1: distance/DrivetrainConstants.pointTolerance;
        }
        VxSB = V.x;
        VySB = V.y;
        setDrivetrain(V.x*scaleFactor*-1, V.y*scaleFactor*-1);
    }
    private void drivePositionRot(){
        updateRotDistance();

        double vr = Math.abs(rotDistance)>DrivetrainConstants.rotTolerance?
        Math.signum(rotDistance): rotDistance/(DrivetrainConstants.decreaseRateRot);
        if(rotDistance <= DrivetrainConstants.validRotDiff) vr = 0;
        setDrivetrainRot(vr);
    }
//-------------------------------------Set Drivetrain based on Drive Functions-----------------------------

    //this allows translation and rotation to be seperated
    private void setDrivetrain(double vx, double vy){
        double scale = DrivetrainConstants.defaultMaxSpeed;
        // if(AreaEffectsHandler.isAreaEffect() == false || AreaEffectsHandler.getMaxSpeed() == null)
        //     scale = DrivetrainConstants.defaultMaxSpeed;
        // else 
            // scale = AreaEffectsHandler.getMaxSpeed();
        appliedX = vx*scale;
        appliedY = vy*scale;

    }
    private void setDrivetrainRot(double vr){
        appliedR = vr*DrivetrainConstants.defaultRotSpeed;
    }
//-------------------------------------------Apply Set Values------------------------------------
    private void applyDrivetrain(){
        Rotation2d currentRotation = PositionComponent.getRobotPose().getRotation();
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(appliedX, appliedY, appliedR, currentRotation);
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        
        for(int i =0; i<4; i++){
            Rotation2d currentAngle = new Rotation2d(maxSwerveModules[i].turningEncoder.getPosition());
            moduleStates[i].optimize(currentAngle);
            moduleStates[i].speedMetersPerSecond *= moduleStates[i].angle.minus(currentAngle).getCos();
        }

        // for(int i = 0; i< 4; i++){
        //     maxSwerveModules[i].turningPIDController.setReference(moduleStates[i].angle.getRadians(), ControlType.kPosition);
        //     maxSwerveModules[i].drivingPIDController.setReference(moduleStates[i].speedMetersPerSecond, ControlType.kVelocity);
        // }
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
        return distance < DrivetrainConstants.validRange;
    }
    public boolean getIsRotationReached(){
        return rotDistance < DrivetrainConstants.validRotDiff;
    }
     public static SwerveModulePosition[] getSwerveModulePositions(){
        SwerveModulePosition[] swerveModulePositionList = {
            // maxSwerveModules[0].getPosition(),
            // maxSwerveModules[1].getPosition(),
            // maxSwerveModules[2].getPosition(),
            // maxSwerveModules[3].getPosition()
        };
            
        return swerveModulePositionList;
    }
//-------------------------------------------Misc Methods------------------------------------
    private void updateDistance(){
        Pose2d currentRobotPosition = PositionComponent.getRobotPose();
        distance = 
        Math.sqrt(
        Math.pow(currentRobotPosition.getX() - posX,2) +
        Math.pow(currentRobotPosition.getY() - posY, 2));
        distanceShuffle =  distance;
    }
    private void updateRotDistance(){
        double currentAngle = PositionComponent.getRobotPose().getRotation().getRadians();
        rotDistance = mod(posR - currentAngle -Math.PI, 2*Math.PI) - Math.PI;
    }
    public boolean checkIsRobotStopped(){
        for(int i =0; i<4; i++){
          if(maxSwerveModules[i].drivingEncoder.getVelocity() > DrivetrainConstants.stoppedVelocity) return false;
        }
        return true;
    }
    public double[] convertToNWU(double x, double y, double r){
        return new double[] {y, -x, r};
    
    }
    public double mod(double a, double b){
        return (a%b + b)%b;
    }
}
