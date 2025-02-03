package frc.robot.Drivetrain;

import java.util.Map;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  //ALEKS: Trapezoidal Motion Profiling to smooth movement
  private final TrapezoidProfile.Constraints m_constraints = 
    new TrapezoidProfile.Constraints(getControllerX(), getControllerRot());
  private final ProfiledPIDController m_controller = 
    new ProfiledPIDController(getControllerY(), getControllerX(), getControllerRot(), m_constraints);
  private final ElevatorFeedforward m_feedforward = 
    new ElevatorFeedforward(getControllerY(), getControllerX(), getControllerRot()); 
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

  public Drivetrain() {
    rotSens = Shuffleboard.getTab("Drivetrain")
    .add("Rotation Sensitivity", rotationSensitivityDefault)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0.3, "max", 2.5))
    .getEntry();
    //ALEKS: Setting the encoder pulse
    m_encoder.setDistancePerPulse(1.0 / 360.0 * 2.0 * Math.PI * 1.5);
  }
      
  
  public static Drivetrain getInstance(){return drivetrain;}

  public void exampleMethodCommand() {}
  
  public void setGoal(float v) {
    m_controller.setGoal(v);
  }

  @Override
  public void periodic() {
    if (m_joystick.getRawButtonPressed(2)) {
      m_controller.setGoal(5);
    elseif (m_joystick.getRawButtonPressed(3)) {
      m_controller.setGoal(0);
    }
    m_motor.setTargetVel(
      m_controller.calculate(m_encoder.getRadians) + m_feedforward.calculate(m_controller.getSetpoint().velocity)
    );
    }


  //ALEKS: ^ Hopefully completed     
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

