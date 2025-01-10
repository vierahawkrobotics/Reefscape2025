package frc.robot.Drivetrain;

import java.util.Map;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  XboxController controller = new XboxController(DrivetrainConstants.usbPortController);
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

  public static double rotationSensitivityDefault = 1;
  
  public static GenericEntry rotSens =
    Shuffleboard.getTab("Drivetrain")
    .add("Rotation Sensitivity", rotationSensitivityDefault)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0.3, "max", 2.5))
    .getEntry();
  
  //Code to make sure Drivetrain only has one instance, please use getInstance to get an instance of Drivetrain
  private static final Drivetrain drivetrain = new Drivetrain();
  private Drivetrain() {}
  
  public static Drivetrain getInstance(){return drivetrain;}

  public void exampleMethodCommand() {}
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  
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
    //Not Updating code
    double rotationSensitivity = Drivetrain.rotSens.getDouble(Drivetrain.rotationSensitivityDefault);
    
    double controllerRot = controller.getRightX();
    controllerRot = (controllerRot< DrivetrainConstants.inputDeadband)?controllerRot=0: controllerRot*rotationSensitivity;
    return controllerRot;
  }
  
  public void setDesiredStates(SwerveModuleState[] desiredStates){
    for(int i = 0; i< 4; i++){
       maxSwerveModules[i].turningPIDController.setReference(desiredStates[i].angle.getRadians(), ControlType.kPosition);
       maxSwerveModules[i].drivingPIDController.setReference(desiredStates[i].speedMetersPerSecond, ControlType.kVelocity);
     }
    }
  
}
