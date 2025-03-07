package frc.robot.Drivetrain;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;



public class MAXSwerveModule{
    private SparkFlex drivingMotorController;
    private SparkMax turningMotorController;
    public SparkClosedLoopController drivingPIDController;
    public SparkClosedLoopController turningPIDController;
    public RelativeEncoder drivingEncoder;
    public AbsoluteEncoder turningEncoder;
    private double chassisAngularOffset;

    public MAXSwerveModule(int drivingMotorID,int turningMotorID,double chassisAngularOffset){

      drivingMotorController = new SparkFlex(drivingMotorID, MotorType.kBrushless);
      turningMotorController = new SparkMax(turningMotorID, MotorType.kBrushless);
      drivingPIDController = drivingMotorController.getClosedLoopController();
      turningPIDController = turningMotorController.getClosedLoopController();
      drivingEncoder = drivingMotorController.getEncoder();
      turningEncoder = turningMotorController.getAbsoluteEncoder();
      this.chassisAngularOffset = chassisAngularOffset;

      SparkMaxConfig turningConfig = new SparkMaxConfig();
      turningConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(DrivetrainConstants.turningMotorCurrentLimit);
      turningConfig.absoluteEncoder
      //.zeroOffset(chassisAngularOffset / (2 * Math.PI))
      .inverted(true)
      .positionConversionFactor(DrivetrainConstants.turningEncoderPositionFactor)
      .velocityConversionFactor(DrivetrainConstants.turningEncoderVelocityFactor);
      turningConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .positionWrappingEnabled(true)
      .positionWrappingInputRange(DrivetrainConstants.turningPIDMinInput, DrivetrainConstants.turningPIDMaxInput)
      .outputRange(DrivetrainConstants.turningMinOutput, DrivetrainConstants.turningMaxOutput)
      .pid(DrivetrainConstants.turningP, DrivetrainConstants.turningI, DrivetrainConstants.turningD);

      SparkMaxConfig drivingConfig = new SparkMaxConfig();
      drivingConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(DrivetrainConstants.drivingMotorCurrentLimit);
      drivingConfig.encoder
      .positionConversionFactor(DrivetrainConstants.drivingEncoderPositionFactor)
      .velocityConversionFactor(DrivetrainConstants.drivingEncoderVelocityFactor);
      drivingConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .outputRange(DrivetrainConstants.drivingMinOutput, DrivetrainConstants.drivingMaxOutput)
      .pid(DrivetrainConstants.drivingP, DrivetrainConstants.drivingI, DrivetrainConstants.drivingD);
      
      drivingMotorController.configure(drivingConfig, DrivetrainConstants.drivingReset, DrivetrainConstants.drivingPersist);
      turningMotorController.configure(turningConfig, DrivetrainConstants.turningReset, DrivetrainConstants.turningPersist);
    }
    public SwerveModulePosition getPosition() {
      // Apply chassis angular offset to the encoder position to get the position
      // relative to the chassis.
      return new SwerveModulePosition(
          drivingEncoder.getPosition(),
          new Rotation2d(turningEncoder.getPosition()));
    }
    public void set(double metersPerSec, double targetRat) {

    }
  
  }


