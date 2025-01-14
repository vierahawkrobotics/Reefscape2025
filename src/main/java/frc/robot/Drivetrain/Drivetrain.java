package frc.robot.Drivetrain;

import java.util.Map;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  //TO DO: this can be changed later for area effects etc, note it must be meters/second
  double maxSpeed = 4;
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
  public static double rotationSensitivityDefault = .9;
  
  public GenericEntry rotSens;

  //double velX,velY;
  //double velR;
  //double posX, posY;
  //double posR;

  
  //Code to make sure Drivetrain only has one instance, please use getInstance to get an instance of Drivetrain
  private static final Drivetrain drivetrain = new Drivetrain();

  private Drivetrain() {
    rotSens = Shuffleboard.getTab("Drivetrain")
    .add("Rotation Sensitivity", rotationSensitivityDefault)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0.3, "max", 2.5))
    .getEntry();
  }
  
  public static Drivetrain getInstance(){return drivetrain;}

  public void exampleMethodCommand() {}
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run\
    // rotationSensitivity isn't upating
    rotationSensitivity = rotSens.getDouble(rotationSensitivityDefault);

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
  
  //remove these
  public double getControllerX(){
    double controllerX = (controller.getLeftX() < 0) ? Math.pow(controller.getLeftX(),2)*(-1): Math.pow(controller.getLeftX(),2);
    controllerX = (controllerX< DrivetrainConstants.inputDeadband)?controllerX=0: controllerX*maxSpeed;
    return controllerX;
  }

  public double getControllerY(){
    //takes the controller input for the controller's Y axis, squares it adding back the negative if the input was negative
    double controllerY = (controller.getLeftY() < 0) ? Math.pow(controller.getLeftY(),2)*(-1): Math.pow(controller.getLeftY(),2);
    //applies an input deadband if the input is out of the deadband scale it using the maxSpeed
    controllerY = (controllerY< DrivetrainConstants.inputDeadband) ? controllerY=0: controllerY*maxSpeed;
    return controllerY;
  }

  public double getControllerRot(){
    System.out.println(rotationSensitivity);
    double controllerRot = controller.getRightX();
    controllerRot = (controllerRot< DrivetrainConstants.inputDeadband)?controllerRot=0: controllerRot*rotationSensitivity;
    return controllerRot;
  }

  //make currentRotation a variable within the function not a parameter
  //should be private
  //rename to DriveVelocity
  public void setDrivetrain(double desiredX, double desiredY, double desiredRot, Rotation2d currentRotation){
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredX, desiredY, desiredRot, currentRotation);

    SwerveModuleState[] moduleStates = Drivetrain.kinematics.toSwerveModuleStates(speeds);
    
    for(int i =0; i<4; i++){
      Rotation2d currentAngle = new Rotation2d(maxSwerveModules[i].turningEncoder.getPosition());
      moduleStates[i].optimize(currentAngle);
      moduleStates[i].speedMetersPerSecond *= moduleStates[i].angle.minus(currentAngle).getCos();
    }

    Drivetrain.getInstance().setDesiredStates(moduleStates);
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

