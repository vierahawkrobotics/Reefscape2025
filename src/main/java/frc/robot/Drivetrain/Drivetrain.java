package frc.robot.Drivetrain;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Drivetrain extends SubsystemBase {
  //TO DO: this can be changed later for area effects etc, note it must be meters/second
  double maxSpeed = 4;
  XboxController controller = Robot.instance.controller;

  //TO DO: replace all uses of this variable with the angle from the position subsystem in future
  public Rotation2d currentRobotAngle = new Rotation2d();
  
  //this should ALWAYS be front left, front right, back left, and then back right
  private static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
  DrivetrainConstants.frontLeftLocation, 
  DrivetrainConstants.frontRightLocation, 
  DrivetrainConstants.backLeftLocation, 
  DrivetrainConstants.backRightLocation
  );

  private static MAXSwerveModule[] maxSwerveModules = {
    new MAXSwerveModule(DrivetrainConstants.flDrivingID, DrivetrainConstants.flTurningID),
    new MAXSwerveModule(DrivetrainConstants.frDrivingID,DrivetrainConstants.frTurningID),
    new MAXSwerveModule(DrivetrainConstants.blDrivingID,DrivetrainConstants.blTurningID),
    new MAXSwerveModule(DrivetrainConstants.brDrivingID,DrivetrainConstants.brTurningID)
  };

  public static ShuffleboardTab drivetrainTab = Shuffleboard.getTab("Drivetrain");

  public double rotationSensitivity;
  
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
      //  DrivePosition(posX,posY);
    }

    if (rotState == RotState.position){
      //  DrivePositionRot(posR);
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
    Rotation2d currentRotation = currentRobotAngle;
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredvX, desiredvY, desiredvRot, currentRotation);

    SwerveModuleState[] moduleStates = Drivetrain.kinematics.toSwerveModuleStates(speeds);
    
    for(int i =0; i<4; i++){
      Rotation2d currentAngle = new Rotation2d(maxSwerveModules[i].turningEncoder.getPosition());
      moduleStates[i].optimize(currentAngle);
      moduleStates[i].speedMetersPerSecond *= moduleStates[i].angle.minus(currentAngle).getCos();
    }

    Robot.instance.drivetrain.setDesiredStates(moduleStates);

  }

  //private void DrivePosition()
  //  sets the translation velocities based on the desired position and current pos
  //private void DrivePositionRot()
  //  rotation version of DrivePosition

  public void setTargetVel(double vx, double vy){
    velX = vx;
    velY = vy;
    translateState = TranslateState.velocity;
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

  //use this ONLY for initialization
  private void setDesiredStates(SwerveModuleState[] desiredStates){
    for(int i = 0; i< 4; i++){
       maxSwerveModules[i].turningPIDController.setReference(desiredStates[i].angle.getRadians(), ControlType.kPosition);
       maxSwerveModules[i].drivingPIDController.setReference(desiredStates[i].speedMetersPerSecond, ControlType.kVelocity);
     }
    }
  
}

