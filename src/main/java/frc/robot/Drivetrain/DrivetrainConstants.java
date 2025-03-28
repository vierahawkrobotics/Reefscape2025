package frc.robot.Drivetrain;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.math.geometry.Translation2d;

public class DrivetrainConstants {
    public static double smoothTransitionRate = 0.04; // 0.5 s decay rate
//speed setting- this is just default if no areaEffects are being
    public static double defaultMaxSpeed = 1.3;// 0.25;
    public static double defaultRotSpeed = Math.PI * 1;
//-------------------------------------------Limits and Deadband-------------------------------------------
    public static double inputDeadband = 0.09;
    public static double physicalSpeedLimit = 20; // meters per second
    public static double physicalRotSpeedLimit = Math.PI * 4; // rad per second
    public static int drivingMotorCurrentLimit = 80; // amps
    public static int turningMotorCurrentLimit = 25; // amps

//-------------------------------------------Point Reached Ranges-------------------------------------------
    //The range at which the robot is considered to have reached a position
    public static double validRange = 0.05; //translation in meters
    public static double validRotDiff = 0.03; //rotation in radians
     //the range at which the robot begins to slow down in DrivePosition()
     public static double pointTolerance = 0.5; //meters
     //the rotation version of what's above
     public static double rotTolerance = 0.18;//radians
     /*the speed(meters/second) at which the robot is considered stopped during
     a path, this will be checked on each SwerveModule individually*/
     public static double stoppedVelocity = 0.02;
    
//-------------------------------------Mechanical Information, Reefscaoe Bot-----------------------------

    // Locations for the swerve drive modules relative to the robot center.
    public static Translation2d frontLeftLocation = new Translation2d(0.29845, 0.29845);
    public static Translation2d frontRightLocation = new Translation2d(0.29845, -0.29845);
    public static Translation2d backLeftLocation = new Translation2d(-0.29845, 0.29845);
    public static Translation2d backRightLocation = new Translation2d(-0.29845, -0.29845);

    //IDs for the driving and turning motors
    public static int flDrivingID = 1;
    public static int flTurningID = 2;
    public static int frDrivingID = 3;
    public static int frTurningID = 4;
    public static int blDrivingID = 5;
    public static int blTurningID = 6;
    public static int brDrivingID = 7;
    public static int brTurningID = 8;

    public static final double wheelDiameterMeters = 0.0762;
    public static final double wheelCircumferenceMeters = wheelDiameterMeters * Math.PI;
    public static final double drivingMotorReduction = 5.5;

    public static final double turningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double turningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second
    public static final double drivingEncoderPositionFactor = (wheelDiameterMeters * Math.PI) / drivingMotorReduction; // meters
    public static final double drivingEncoderVelocityFactor = ((wheelDiameterMeters * Math.PI) / drivingMotorReduction) / 60.0; // meters per second

    //------------------------------------------------PIDs-------------------------------------------------
    public static double drivingP = 0.6;
    public static double drivingI = 0;
    public static double drivingD = 0.06;
    public static double drivingMinOutput = -1;
    public static double drivingMaxOutput = 1;
    public static PersistMode drivingPersist = PersistMode.kNoPersistParameters;
    public static ResetMode drivingReset = ResetMode.kResetSafeParameters;

    public static double turningP = 0.3;
    public static double turningI = 0;
    public static double turningD = 0.05;
    //sets wrapping for the PIDController
    public static double turningPIDMinInput = 0; // radians
    public static double turningPIDMaxInput = turningEncoderPositionFactor; // radians
    public static double turningMinOutput = -1;
    public static double turningMaxOutput = 1;

    //------------------------------------------------Misc Settings-------------------------------------------------
    public static PersistMode turningPersist = PersistMode.kNoPersistParameters;
    public static ResetMode turningReset = ResetMode.kResetSafeParameters;

    public static final double flChassisAngularOffset = 0;
    public static final double frChassisAngularOffset = 0;
    public static final double blChassisAngularOffset = 0;
    public static final double brChassisAngularOffset = 0;

    //-------------------------------------Mechanical Information, Crescendo Bot-----------------------------

    // // Locations for the swerve drive modules relative to the robot center.
    // public static Translation2d frontLeftLocation = new Translation2d(0.29845, 0.29845);
    // public static Translation2d frontRightLocation = new Translation2d(0.29845, -0.29845);
    // public static Translation2d backLeftLocation = new Translation2d(-0.29845, 0.29845);
    // public static Translation2d backRightLocation = new Translation2d(-0.29845, -0.29845);

    // //IDs for the driving and turning motors
    // public static int flDrivingID = 3;
    // public static int flTurningID = 7;
    // public static int frDrivingID = 2;
    // public static int frTurningID = 6;
    // public static int blDrivingID = 4;
    // public static int blTurningID = 8;
    // public static int brDrivingID = 1;
    // public static int brTurningID = 5;

    // public static final double wheelDiameterMeters = 0.074;
    // public static final int drivingMotorPinionTeeth = 14;
    // public static final int drivingMotorSpurTeeth = 21;
    // public static final double wheelCircumferenceMeters = wheelDiameterMeters * Math.PI;
    // public static final double drivingMotorReduction = (45.0 * drivingMotorSpurTeeth) / (drivingMotorPinionTeeth * 15);

    // public static final double turningEncoderPositionFactor = (2 * Math.PI); // radians
    // public static final double turningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second
    // public static final double drivingEncoderPositionFactor = (wheelDiameterMeters * Math.PI) / drivingMotorReduction; // meters
    // public static final double drivingEncoderVelocityFactor = ((wheelDiameterMeters * Math.PI) / drivingMotorReduction) / 60.0; // meters per second

}




