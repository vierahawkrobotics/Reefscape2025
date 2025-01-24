package frc.robot.Drivetrain;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;



public class MAXSwerveModule{
    SparkFlex drivingMotorController;
    SparkMax turningMotorController;
    SparkClosedLoopController drivingPIDController;
    SparkClosedLoopController turningPIDController;
    RelativeEncoder drivingEncoder;
    AbsoluteEncoder turningEncoder;

    public MAXSwerveModule(int drivingMotorID,int turningMotorID){

      drivingMotorController = new SparkFlex(drivingMotorID, MotorType.kBrushless);
      turningMotorController = new SparkMax(turningMotorID, MotorType.kBrushless);
      drivingPIDController = drivingMotorController.getClosedLoopController();
      turningPIDController = turningMotorController.getClosedLoopController();
      drivingEncoder = drivingMotorController.getEncoder();
      turningEncoder = turningMotorController.getAbsoluteEncoder();

      SparkMaxConfig turningConfig = new SparkMaxConfig();
      turningConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(DrivetrainConstants.turningMotorCurrentLimit);
      turningConfig.absoluteEncoder
      .inverted(true)
      .positionConversionFactor(DrivetrainConstants.turningEncoderPositionFactor)
      .velocityConversionFactor(DrivetrainConstants.turningEncoderVelocityFactor);
      turningConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
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
  
  }


