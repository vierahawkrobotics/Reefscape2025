package frc.robot.Drivetrain;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Components.PositionComponent.PositionComponent;

public class Drivetrain extends SubsystemBase {
  //TO DO: this can be changed later for area effects etc, note it must be meters/second
  double maxSpeed = 1;
  double distance = 0;
  double rotDistance = 0;

  //this should ALWAYS be front left, front right, back left, and then back right
  public static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
  DrivetrainConstants.frontLeftLocation, 
  DrivetrainConstants.frontRightLocation, 
  DrivetrainConstants.backLeftLocation, 
  DrivetrainConstants.backRightLocation
  );

  public static MAXSwerveModule[] maxSwerveModules = {
    new MAXSwerveModule(DrivetrainConstants.flDrivingID,DrivetrainConstants.flTurningID,DrivetrainConstants.flChassisAngularOffset),
    new MAXSwerveModule(DrivetrainConstants.frDrivingID,DrivetrainConstants.frTurningID,DrivetrainConstants.frChassisAngularOffset),
    new MAXSwerveModule(DrivetrainConstants.blDrivingID,DrivetrainConstants.blTurningID,DrivetrainConstants.blChassisAngularOffset),
    new MAXSwerveModule(DrivetrainConstants.brDrivingID,DrivetrainConstants.brTurningID,DrivetrainConstants.brChassisAngularOffset)
  };

  public static ShuffleboardTab drivetrainTab = Shuffleboard.getTab("Drivetrain");
  
  enum TranslateState{
    velocity,
    position,
    path
  }

  enum RotState{
    velocity,
    position,
    path
  }

  TranslateState translateState = TranslateState.velocity;
  RotState rotState = RotState.velocity;

  StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
.getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
  
  private double velX;
  private double velY;
  private double velR;
  private double posX;
  private double posY;
  private double posR;
  //targetX, Y, optional R, check velocity zero, radius factor
  private Path path;

  ShuffleboardTab drivetrainPIDTab = Shuffleboard.getTab("Drivetrain PIDs");
  ShuffleboardLayout drivingPIDs = drivetrainPIDTab
    .getLayout("Driving PID Constants", BuiltInLayouts.kList)
    .withSize(2,2);
  ShuffleboardLayout turningPIDs = drivetrainPIDTab
  .getLayout("Turning PID Constants", BuiltInLayouts.kList)
  .withSize(2,2);

  public Drivetrain() {
      drivingPIDs.add("P", DrivetrainConstants.drivingP);
      drivingPIDs.add("I", DrivetrainConstants.drivingI);
      drivingPIDs.add("D", DrivetrainConstants.drivingD);

      turningPIDs.add("P", DrivetrainConstants.turningP);
      turningPIDs.add("I", DrivetrainConstants.turningI);
      turningPIDs.add("D", DrivetrainConstants.turningD);

      drivetrainTab.addDouble("Robot rot", () -> {return PositionComponent.getRobotPose().getRotation().getRadians(); });

      drivetrainTab.addDouble("Robot velR", () -> {return velR;});
      drivetrainTab.addDouble("Robot velX", () -> {return velX;});
      drivetrainTab.addDouble("Robot velY", () -> {return velY;});
  }

  @Override
  public void periodic() {

    if (translateState == TranslateState.position){
      DrivePosition();
    }
<<<<<<< Updated upstream
    else if (translateState == TranslateState.path){
      
      if(distance < DrivetrainConstants.validRange*path.getRadiusFactor() && 
      (path.getCurrentRot() == null || rotDistance < DrivetrainConstants.validRotDiff) && 
      (path.getVelocityCheckSetting()== false || checkIsRobotStopped()))
      {
        path.increaseIndex();
        if(path.getPathStatus() == true){
          translateState = TranslateState.velocity;
          rotState = RotState.velocity;
=======
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

    private Path path;
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
        switch(translateState){
            case path:
                drivePath();
                break;
            case velocity:
                driveVelocity();
                break;
            case position:
                drivePosition();
                break;
>>>>>>> Stashed changes
        }
        else {
          posX = path.getCurrentX();
          posY = path.getCurrentY();
          posR = path.getCurrentPoint() == null? PositionComponent.getRobotPose().getRotation().getRadians(): path.getCurrentRot();
          DrivePosition();
          DrivePositionRot();
        }  
      }
      else{
        DrivePosition();
      }
    }
<<<<<<< Updated upstream

    if (rotState == RotState.position){
      DrivePositionRot();
=======
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
        double[] velocitiesAndScaleFactor = getVelocityForPos(posX, posY);
        setDrivetrain(velocitiesAndScaleFactor[0]*velocitiesAndScaleFactor[2],
         velocitiesAndScaleFactor[1]*velocitiesAndScaleFactor[2]);
    }
    private void drivePositionRot(){
        updateRotDistance();
        setDrivetrainRot(getVelocityForPosRot(posR));
    }
    private void drivePath(){
      updateDistance();
      updateRotDistance();
      //if the robot has reached the point, go to the next point
      if(distance < DrivetrainConstants.validRange*path.getRadiusFactor() && 
      (path.getCurrentRot() == null || rotDistance < DrivetrainConstants.validRotDiff) && 
      (path.getVelocityCheckSetting()== false || checkIsRobotStopped()))
      {
        path.increaseIndex();
        }
      //if the path is still on going, set robot based on the x,y,and r of the next point
      if(!(path.getIsPathFinished())){
        double[] velocitiesAndScaleFactor = getVelocityForPos(path.getCurrentX(), path.getCurrentY());
        setDrivetrain(velocitiesAndScaleFactor[0]*velocitiesAndScaleFactor[2],
          velocitiesAndScaleFactor[1]*velocitiesAndScaleFactor[2]);
        setDrivetrainRot(getVelocityForPosRot(posR));
      }
    }
//----------------------------Intermediete Step for DrivePosition and DrivePath----------------------------
    private double[] getVelocityForPos(double x, double y){
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
        return new double[] {V.x, V.y, scaleFactor};
    }
    private double getVelocityForPosRot(double r){

        double vr = Math.abs(rotDistance)>DrivetrainConstants.rotTolerance?
        Math.signum(rotDistance): rotDistance/(DrivetrainConstants.decreaseRateRot);
        if(rotDistance <= DrivetrainConstants.validRotDiff) vr = 0;
        return vr;
>>>>>>> Stashed changes
    }

<<<<<<< Updated upstream
    DriveVelocity(velX, velY, velR);
=======
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
    public void setPath(Path newPath){
      path = newPath;
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
>>>>>>> Stashed changes
    
    
    velX = 0;
    velY = 0;
    velR = 0;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  private void DriveVelocity(double desiredvX, double desiredvY, double desiredvRot){
    Rotation2d currentRotation = PositionComponent.getRobotPose().getRotation();
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredvX, desiredvY, desiredvRot, currentRotation);
    SwerveModuleState[] moduleStates = Drivetrain.kinematics.toSwerveModuleStates(speeds);
    
    for(int i =0; i<4; i++){
      Rotation2d currentAngle = new Rotation2d(maxSwerveModules[i].turningEncoder.getPosition());
      moduleStates[i].optimize(currentAngle);
      moduleStates[i].speedMetersPerSecond *= moduleStates[i].angle.minus(currentAngle).getCos();
    }
<<<<<<< Updated upstream

    setDesiredStates(moduleStates);

  }

  private void UpdateDistance() {
    Pose2d currentRobotPosition = PositionComponent.getRobotPose();
    distance = 
    Math.sqrt(
    Math.pow(currentRobotPosition.getX() - posX,2) +
    Math.pow(currentRobotPosition.getY() - posY, 2));
  }

  public boolean checkIsRobotStopped(){
    for(int i =0; i<4; i++){
      if(maxSwerveModules[i].drivingEncoder.getVelocity() > DrivetrainConstants.stoppedVelocity) return false;
=======
    public double mod(double a, double b){
      return (a%b + b)%b;
>>>>>>> Stashed changes
    }
    return true;
  }

  private void DrivePosition(){
    Pose2d currentRobotPosition = PositionComponent.getRobotPose();
    //robot current position
    Vector R = new Vector(currentRobotPosition.getX(), currentRobotPosition.getY());
    //target position
    Vector T = new Vector(posX, posY);

    UpdateDistance();

    //normal vector
    Vector V = (T.subtract(R)).normalize();
    double scaleFactor = distance>DrivetrainConstants.pointTolerance? 1: distance/DrivetrainConstants.pointTolerance;
    setTargetVel(V.x*scaleFactor, V.y*scaleFactor, false);
  }
  private void DrivePositionRot(){
    double angle = PositionComponent.getRobotPose().getRotation().getRadians();
    rotDistance = angle - posR > 0? posR-angle: angle- posR;
    velR = rotDistance>DrivetrainConstants.rotTolerance? 1: rotDistance/DrivetrainConstants.rotTolerance;
  }

  public void setPath(Path pathInput, Supplier<Boolean> booleanSupplier){
    // path[0][0] = 3;
    path = pathInput;
    translateState = TranslateState.path;
    rotState = RotState.path;
    
    posX = path.getCurrentX();
    posY = path.getCurrentY();
    posR = path.getCurrentPoint() == null? PositionComponent.getRobotPose().getRotation().getRadians(): path.getCurrentRot();
    }
    

  //vx and vy should be from 0 to 1
  public void setTargetVel(double vx, double vy, boolean setToVelMode){
    velX = vx*maxSpeed;
    velY = vy*maxSpeed;
    if(setToVelMode){translateState = TranslateState.velocity;}
  }

  public void setTargetPos(double pX, double pY){
    posX = pX;
    posY = pY;
    translateState = TranslateState.position;
  }

  public void setTargetVelRot(double vr){
    velR = vr;
    rotState = RotState.velocity;
  }

  public void setTargetPosRot(double pR){
    posR = pR;
    rotState = RotState.position;
  }

  private void setDesiredStates(SwerveModuleState[] desiredStates){
    for(int i = 0; i< 4; i++){
       maxSwerveModules[i].turningPIDController.setReference(desiredStates[i].angle.getRadians(), ControlType.kPosition);
       maxSwerveModules[i].drivingPIDController.setReference(desiredStates[i].speedMetersPerSecond, ControlType.kVelocity);
     }
    publisher.set(desiredStates);
  }
    
  public static SwerveModulePosition[] getSwerveModulePositions(){
    SwerveModulePosition[] swerveModulePositionList = {
      maxSwerveModules[0].getPosition(),
      maxSwerveModules[1].getPosition(),
      maxSwerveModules[2].getPosition(),
      maxSwerveModules[3].getPosition(),
    };

    return swerveModulePositionList;
  }
  
}
