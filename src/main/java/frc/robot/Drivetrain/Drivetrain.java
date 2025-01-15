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
  public static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
  DrivetrainConstants.frontLeftLocation, 
  DrivetrainConstants.frontRightLocation, 
  DrivetrainConstants.backLeftLocation, 
  DrivetrainConstants.backRightLocation
  );

  public static MAXSwerveModule[] maxSwerveModules = {
    new MAXSwerveModule(DrivetrainConstants.flDrivingID, DrivetrainConstants.flTurningID),
    new MAXSwerveModule(DrivetrainConstants.frDrivingID,DrivetrainConstants.frTurningID),
    new MAXSwerveModule(DrivetrainConstants.blDrivingID,DrivetrainConstants.blTurningID),
    new MAXSwerveModule(DrivetrainConstants.brDrivingID,DrivetrainConstants.brTurningID)
  };

  public static ShuffleboardTab drivetrainTab = Shuffleboard.getTab("Drivetrain");

  public double rotationSensitivity;
  

  //double velX,velY;
  //double velR;
  //double posX, posY;
  //double posR;

  
  public Drivetrain() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run\

    //switch xyState:
    //velocity:
    //  DriveVelocity(velX,velY);
    //  break;
    //position:
    //  DrivePosition(posX,posY);
    //  break;

    //switch rotState:
    //velocity:
    //  DriveVelocityRot(velR);
    //  break;
    //position:
    //  DrivePositionRot(posR);
    //  break;

    //velX,velY,velR = 0;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setDrive3d(double vx, double vy, double vr){
    //THis will get replaced with setting the velocities that get checked and 
    //used in the periodic function
    DriveVelocity(vx, vy, vr);
  }

  private void DriveVelocity(double desiredvX, double desiredvY, double desiredRot){
    Rotation2d currentRotation = currentRobotAngle;
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredvX, desiredvY, desiredRot, currentRotation);

    SwerveModuleState[] moduleStates = Drivetrain.kinematics.toSwerveModuleStates(speeds);
    
    for(int i =0; i<4; i++){
      Rotation2d currentAngle = new Rotation2d(maxSwerveModules[i].turningEncoder.getPosition());
      moduleStates[i].optimize(currentAngle);
      moduleStates[i].speedMetersPerSecond *= moduleStates[i].angle.minus(currentAngle).getCos();
    }

    Robot.instance.drivetrain.setDesiredStates(moduleStates);
  }

  //private void DrivePosition(posX,posY)
  //private void DriveVelocityRot(velR)
  //private void DrivePositionRot(posR)

  //public void setTargetVel()
  //  change state
  //public void setTargetPos()
  //  change state

  //public void setTargetVelRot()
  //  change state
  //public void setTargetPosRot()
  //  change state

  //use this ONLY for initialization
  private void setDesiredStates(SwerveModuleState[] desiredStates){
    for(int i = 0; i< 4; i++){
       maxSwerveModules[i].turningPIDController.setReference(desiredStates[i].angle.getRadians(), ControlType.kPosition);
       maxSwerveModules[i].drivingPIDController.setReference(desiredStates[i].speedMetersPerSecond, ControlType.kVelocity);
     }
    }
  
}

