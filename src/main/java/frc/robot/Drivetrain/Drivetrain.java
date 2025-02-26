package frc.robot.Drivetrain;

import java.io.Console;
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
  double maxSpeed = 1.5;
  double maxRotSpeed = Math.PI*(3/2);
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
private double velTX;
private double velTY;
private double velTR;
  private double posX;
  private double posY;
  private double posR;
  private boolean isPointReached = false;
  private boolean isRotationReached = false;
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
      //drivingPIDs.add("P", DrivetrainConstants.drivingP);
      //drivingPIDs.add("I", DrivetrainConstants.drivingI);
      //drivingPIDs.add("D", DrivetrainConstants.drivingD);

      //turningPIDs.add("P", DrivetrainConstants.turningP);
      //turningPIDs.add("I", DrivetrainConstants.turningI);
      //turningPIDs.add("D", DrivetrainConstants.turningD);

      //drivetrainTab.addDouble("Robot rot", () -> {return PositionComponent.getRobotPose().getRotation().getRadians(); });

      drivetrainTab.addDouble("Robot velR", () -> {return velTR;});
      drivetrainTab.addDouble("Robot velX", () -> {return velTX;});
      drivetrainTab.addDouble("Robot velY", () -> {return velTY;});

      drivetrainTab.addDouble("Robot posR", () -> {return posR;});
      drivetrainTab.addDouble("Robot posX", () -> {return posX;});
      drivetrainTab.addDouble("Robot posY", () -> {return posY;});
  }

  @Override
  public void periodic() {
    if (translateState == TranslateState.position){
      DrivePosition();
      System.out.println("DrivePosition() Called");
    }
    else if (translateState == TranslateState.path){
      
      if(distance < DrivetrainConstants.validRange*path.getRadiusFactor() && 
      (path.getCurrentRot() == null || rotDistance < DrivetrainConstants.validRotDiff) && 
      (path.getVelocityCheckSetting()== false || checkIsRobotStopped()))
      {
        path.increaseIndex();
        if(path.getPathStatus() == true){
          translateState = TranslateState.velocity;
          rotState = RotState.velocity;
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

    if (rotState == RotState.position){
      DrivePositionRot();
      System.out.println("DrivePositionRot() Called");
    }

    DriveVelocity(velX, velY, velR);
    
    velTX = velX;
    velTY = velY;
    velTR = velR;
    
    velX = 0;
    velY = 0;
    velR = 0;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  private void DriveVelocity(double desiredvX, double desiredvY, double desiredvRot){
    Rotation2d currentRotation = PositionComponent.getRobotPose().getRotation().times(-1);
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredvX, desiredvY, desiredvRot, currentRotation);
    SwerveModuleState[] moduleStates = Drivetrain.kinematics.toSwerveModuleStates(speeds);
    
    for(int i =0; i<4; i++){
      Rotation2d currentAngle = new Rotation2d(maxSwerveModules[i].turningEncoder.getPosition());
      moduleStates[i].optimize(currentAngle);
      moduleStates[i].speedMetersPerSecond *= moduleStates[i].angle.minus(currentAngle).getCos();
    }

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
    }
    return true;
  }
  public boolean getIsPointReached(){
    return Robot.instance.drivetrain.distance < DrivetrainConstants.validRange;
  }
  public boolean getIsRotationReached(){
    return Robot.instance.drivetrain.rotDistance < DrivetrainConstants.validRotDiff;
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
    System.out.println("Vector Math in DrivePosition done, setTargetVel called: " + V.x + ", " + V.y);
  }
  private void DrivePositionRot(){
    double angle = PositionComponent.getRobotPose().getRotation().getRadians();
    rotDistance = angle - posR > 0? posR-angle: angle- posR;
    double r = rotDistance>DrivetrainConstants.rotTolerance? 1: rotDistance/DrivetrainConstants.rotTolerance;
    setTargetVelRot(r, false);
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
    isPointReached = false;
  }

  public void setTargetVelRot(double vr){
    setTargetVelRot(vr, true);
  }

  public void setTargetVelRot(double vr, boolean setToVelMode){
    velR = vr*maxRotSpeed;
    if(setToVelMode) rotState = RotState.velocity;
  }

  public void setTargetPosRot(double pR){
    posR = pR;
    rotState = RotState.position;
    isRotationReached = false;
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
