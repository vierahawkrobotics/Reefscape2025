package frc.robot.Drivetrain;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Drivetrain extends SubsystemBase {
  //TO DO: this can be changed later for area effects etc, note it must be meters/second
  private double maxSpeed = 3;
  private double maxRotSpeed = 5.5;
  XboxController controller = Robot.instance.controller;

  //TO DO: replace all uses of this variable with the angle from the position subsystem in future
  public Rotation2d currentRobotAngle = new Rotation2d();
  //TO DO: same as above but for position
  public Pose2d currentRobotPosition = new Pose2d();

  public double distance;
  public double rotDistance;

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
    position
  }
  enum RotState{
    velocity,
    position
  }
  TranslateState translateState = TranslateState.velocity;
  RotState rotState = RotState.velocity;
  
  private double velX;
  private double velY;
  private double velR;
  private double posX;
  private double posY;
  private double posR;

  public Drivetrain() {
  }

  @Override
  public void periodic() {

    if (translateState == TranslateState.position){
      DrivePosition();
    }

    if (rotState == RotState.position){
      DrivePositionRot();
    }

    DriveVelocity(velX, velY, velR);
    
    velX = 0;
    velY = 0;
    velR = 0;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  private void DriveVelocity(double desiredvX, double desiredvY, double desiredvRot){
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredvX, desiredvY, desiredvRot, currentRobotAngle);

    SwerveModuleState[] moduleStates = Drivetrain.kinematics.toSwerveModuleStates(speeds);
    
    for(int i =0; i<4; i++){
      Rotation2d currentAngle = new Rotation2d(maxSwerveModules[i].turningEncoder.getPosition());
      moduleStates[i].optimize(currentAngle);
      moduleStates[i].speedMetersPerSecond *= moduleStates[i].angle.minus(currentAngle).getCos();
    }

    Robot.instance.drivetrain.setDesiredStates(moduleStates);

  }

  private void DrivePosition(){
    
    Vector R = new Vector(currentRobotPosition.getX(), currentRobotPosition.getY());
    Vector T = new Vector(posX, posY);

    distance = 
    Math.sqrt(
    Math.pow(currentRobotPosition.getX() - posX,2) +
    Math.pow(currentRobotPosition.getY() - posY, 2));

    Vector V = T.subtract(R).normalize();
    double scaleFactor = distance >DrivetrainConstants.pointTolerance? 1: distance/DrivetrainConstants.pointTolerance;

    setTargetVel(V.x*scaleFactor, V.y*scaleFactor);
  }
  private void DrivePositionRot(){
    double angle = currentRobotAngle.getRadians();

    double absDiff = Math.abs(angle-posR);
    rotDistance = absDiff>Math.PI? 2*Math.PI - absDiff: absDiff;

    velR = rotDistance>DrivetrainConstants.rotTolerance? 1: rotDistance/DrivetrainConstants.rotTolerance;
    if((posR - angle + Math.PI*2) % (Math.PI*2)< Math.PI){velR*=-1;}
  }

  //vx and vy should be from 0 to 1
  public void setTargetVel(double vx, double vy){
    velX = (vx<1) && (vx >-1)? vx*maxSpeed: maxSpeed;
    velY = (vy<1) && (vx >-1)? vy*maxSpeed: maxSpeed;
    translateState = TranslateState.velocity;
  }

  public void setTargetPos(double pX, double pY){
    posX = pX;
    posY = pY;
    translateState = TranslateState.position;
  }

  public void setTargetVelRot(double vr){
    velR = (vr<1) && (vr>-1)? vr*maxRotSpeed: maxRotSpeed;
    rotState = RotState.velocity;
  }

  public void setTargetPosRot(double pR){
    posR = pR;
    rotState = RotState.position;
  }

  public void setMaxSpeed(double desiredSpeed){
    maxSpeed = (desiredSpeed < DrivetrainConstants.physicalSpeedLimit) && (desiredSpeed > 0)? desiredSpeed: DrivetrainConstants.physicalSpeedLimit;
  }

  public void setMaxRotSpeed(double desiredSpeed){
    maxRotSpeed = (desiredSpeed < DrivetrainConstants.physicalRotSpeedLimit) && (desiredSpeed > 0)? desiredSpeed: DrivetrainConstants.physicalRotSpeedLimit;
  }
  //use this ONLY for initialization
  private void setDesiredStates(SwerveModuleState[] desiredStates){
    for(int i = 0; i< 4; i++){
       maxSwerveModules[i].turningPIDController.setReference(desiredStates[i].angle.getRadians(), ControlType.kPosition);
       maxSwerveModules[i].drivingPIDController.setReference(desiredStates[i].speedMetersPerSecond, ControlType.kVelocity);
     }
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

