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
        new MAXSwerveModule(DrivetrainConstants.flDrivingID,DrivetrainConstants
        .flTurningID,DrivetrainConstants.flChassisAngularOffset, true),
        new MAXSwerveModule(DrivetrainConstants.frDrivingID,DrivetrainConstants
        .frTurningID,DrivetrainConstants.frChassisAngularOffset, false),
        new MAXSwerveModule(DrivetrainConstants.blDrivingID,DrivetrainConstants
        .blTurningID,DrivetrainConstants.blChassisAngularOffset, true),
        new MAXSwerveModule(DrivetrainConstants.brDrivingID,DrivetrainConstants
        .brTurningID,DrivetrainConstants.brChassisAngularOffset, true)
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

    //are we using inputVel or vel with speed already added?
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
    public double smoothScale = DrivetrainConstants.defaultMaxSpeed;
    private double smoothScaleRot = DrivetrainConstants.defaultRotSpeed;

    ShuffleboardTab drivetrainTab = Shuffleboard.getTab("Drivetrain");
    public Drivetrain(){
        //invert back two driving motors

        // drivetrainTab.addDouble("Robot velR", () -> {return velTR;});
        // drivetrainTab.addDouble("Robot velX", () -> {return velTX;});
        // drivetrainTab.addDouble("Robot velY", () -> {return velTX;});
        // drivetrainTab.addDouble("Robot velY", () -> {return velTY;});

        // drivetrainTab.addDouble("Robot posR", () -> {return posR;});
        // drivetrainTab.addDouble("Robot posX", () -> {return posX;});
        // drivetrainTab.addDouble("Robot posY", () -> {return posY;});

        // drivetrainTab.addDouble("Set X Speed", () -> {return setVelX;});
        // drivetrainTab.addDouble("Set Y Speed", () -> {return setVelY;});
        // drivetrainTab.addDouble("V.x", () -> {return VxSB;});
        // drivetrainTab.addDouble("V.y", () -> {return VySB;});
        drivetrainTab.addDouble("a.x", () -> {return appliedX;});
        drivetrainTab.addDouble("a.y", () -> {return appliedY;});
        drivetrainTab.addDouble("a.r", () -> {return appliedR;});
        // drivetrainTab.addDouble("Distance From Point", () -> {return distanceShuffle;});
        // drivetrainTab.addDouble("Rot Distance From Point", () -> {return rotDistance;});
        // drivetrainTab.addDouble("Smooth vel", () -> {return smoothScale;});
        // drivetrainTab.addString("tran state", () -> {return translateState.toString();});
    }
//-------------------------------------------Periodic------------------------------------  
    @Override 
    public void periodic(){
        double scale = DrivetrainConstants.defaultMaxSpeed;
        Double d = AreaEffectsHandler.getMaxSpeed();
        if(AreaEffectsHandler.isAreaEffect() && d != null) {
            scale = d;
        }
        smoothScale = smoothScale + DrivetrainConstants.smoothTransitionRate * (scale - smoothScale);

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
    }
//-------------------------------------------Drive Functions------------------------------------
    private void driveVelocity(){
        updateDistance();
        setDrivetrain(velX, velY);
        velTX = velX;
        velTY = velY;
        velX = 0;
        velY = 0;
    }
    private void driveVelocityRot(){
        setDrivetrainRot(velR);
        velTR = velR;
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
        // if (distance <= DrivetrainConstants.validRange) scaleFactor = 0;
        // else{
        //     scaleFactor = distance>DrivetrainConstants.pointTolerance? 1: distance/DrivetrainConstants.pointTolerance;
        // }
        double pt = smoothScale * DrivetrainConstants.pointTolerance;
        scaleFactor = distance>pt? 1: distance/pt;
        V.x *= smoothScale;
        V.y *= smoothScale;
        VxSB = V.x;
        VySB = V.y;
        setDrivetrain(V.x*scaleFactor, V.y*scaleFactor);
    }
    private void drivePositionRot(){
        double vr;
        updateRotDistance();

        // if(Math.abs(rotDistance) <= DrivetrainConstants.validRotDiff) vr = 0;
        // else{
        //     vr = Math.abs(rotDistance)>DrivetrainConstants.rotTolerance?
        //     Math.signum(rotDistance): rotDistance/(DrivetrainConstants.decreaseRateRot);
        // }
        double rt = smoothScaleRot * DrivetrainConstants.rotTolerance;
        vr = Math.abs(rotDistance)> rt?
        Math.signum(rotDistance): rotDistance/rt;
        setDrivetrainRot(vr * smoothScaleRot);
    }
//-------------------------------------Set Drivetrain based on Drive Functions-----------------------------
    
    //this allows translation and rotation to be seperated
    private void setDrivetrain(double vx, double vy){
        appliedX = Math.abs(vx) > DrivetrainConstants.physicalSpeedLimit? Math.signum(vx)*DrivetrainConstants.physicalSpeedLimit: vx;
        appliedY = Math.abs(vy) > DrivetrainConstants.physicalSpeedLimit? Math.signum(vy)*DrivetrainConstants.physicalSpeedLimit: vy;

    }
    private void setDrivetrainRot(double vr){
        appliedR = Math.abs(vr) > DrivetrainConstants.physicalSpeedLimit? Math.signum(vr)*DrivetrainConstants.physicalSpeedLimit: vr;
    }
//-------------------------------------------Apply Set Values------------------------------------
    private void applyDrivetrain(){

        Rotation2d currentRotation = PositionComponent.getRobotPose().getRotation();
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(appliedX,  appliedY, appliedR, currentRotation);
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        
        for(int i = 0; i < 4; i++){
            Rotation2d currentAngle = new Rotation2d(maxSwerveModules[i].turningEncoder.getPosition());
            moduleStates[i].optimize(currentAngle);
            moduleStates[i].speedMetersPerSecond *= moduleStates[i].angle.minus(currentAngle).getCos();
        }

         for(int i = 0; i < 4; i++){
            maxSwerveModules[i].turningPIDController.setReference(moduleStates[i].angle.getRadians(), ControlType.kPosition);
            maxSwerveModules[i].drivingPIDController.setReference(moduleStates[i].speedMetersPerSecond, ControlType.kVelocity);
        }
    }
//------------------------------------------Setter Methods------------------------------------
   
    //Sets the velocity from -1 to 1. Multiplies it by the max speed later
    public void setInputVel(double vx, double vy){
        if (vx > 1) vx = 1;
        else if (vx < -1) vx = -1;
        velX = vx * smoothScale;
        if (vy > 1) vy = 1;
        else if (vy < -1) vy = -1;
        velY = vy * smoothScale;
        
        translateState = TranslateState.velocity;
    }
    public void setInputVelRot(double vr){
        if (vr > 1) vr = 1;
        else if (vr < -1) vr = -1;
        velR = vr * smoothScaleRot;
        rotationState = RotationState.velocity;
    }
    //Sets the velocity including the speed.
    public void setVelocity(double vx, double vy){
        velX = vx;
        velY = vy;
        translateState = TranslateState.velocity;
    }
    public void setVelocityRot(double vr){
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
    
    public boolean getIsPointReached(double customRadiusMeters){
        return distance < customRadiusMeters;
    }
    public boolean getIsRotationReached(){
        return Math.abs(rotDistance) < DrivetrainConstants.validRotDiff;
    }
    public boolean getIsRotationReached(double customRadiusRad){
        return Math.abs(rotDistance) < customRadiusRad;
    }
     public static SwerveModulePosition[] getSwerveModulePositions(){
        SwerveModulePosition[] swerveModulePositionList = {
        maxSwerveModules[0].getPosition(),
        maxSwerveModules[1].getPosition(),
        maxSwerveModules[2].getPosition(),
        maxSwerveModules[3].getPosition()};
        
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
        rotDistance = posR - currentAngle;
        double k = 2 * Math.PI + posR - currentAngle;
        if(Math.abs(k) < Math.abs(rotDistance)) rotDistance = k;
        k = -2 * Math.PI + posR - currentAngle;
        if(Math.abs(k) < Math.abs(rotDistance)) rotDistance = k;
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