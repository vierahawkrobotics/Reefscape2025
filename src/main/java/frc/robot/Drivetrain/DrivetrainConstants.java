package frc.robot.Drivetrain;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.math.geometry.Translation2d;

public class DrivetrainConstants {
    public static int usbPortController = 0;
    public static double inputDeadband = 0.02;
    // Locations for the swerve drive modules relative to the robot center.
    public static Translation2d frontLeftLocation = new Translation2d(0.381, 0.381);
    public static Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
    public static Translation2d backLeftLocation = new Translation2d(-0.381, 0.381);
    public static Translation2d backRightLocation = new Translation2d(-0.381, -0.381);

    //IDs for the driving and turning motors
    public static int flDrivingID = 3;
    public static int flTurningID = 7;
    public static int frDrivingID = 2;
    public static int frTurningID = 6;
    public static int blDrivingID = 4;
    public static int blTurningID = 8;
    public static int brDrivingID = 1;
    public static int brTurningID = 5;

    public static final double wheelDiameterMeters = 6;
    public static final int drivingMotorPinionTeeth = 14;
    public static final int drivingMotorSpurTeeth = 21;
    public static final double wheelCircumferenceMeters = wheelDiameterMeters * Math.PI;
    public static final double drivingMotorReduction = (45.0 * drivingMotorSpurTeeth) / (drivingMotorPinionTeeth * 15);

    public static final double turningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double turningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second
    public static final double drivingEncoderPositionFactor = (wheelDiameterMeters * Math.PI) / drivingMotorReduction; // meters
    public static final double drivingEncoderVelocityFactor = ((wheelDiameterMeters * Math.PI) / drivingMotorReduction) / 60.0; // meters per second

    public static double drivingP = 0.125;
    public static double drivingI = 0;
    public static double drivingD = 0;
    public static double drivingMinOutput = -1;
    public static double drivingMaxOutput = 1;
    public static PersistMode drivingPersist = PersistMode.kNoPersistParameters;
    public static ResetMode drivingReset = ResetMode.kResetSafeParameters;

    public static double turningP = 1.75;
    public static double turningI = 0;
    public static double turningD = 0;
    public static double turningPIDMinInput = 0; // radians
    public static double turningPIDMaxInput = turningEncoderPositionFactor; // radians
    public static double turningMinOutput = -1;
    public static double turningMaxOutput = 1;
    public static PersistMode turningPersist = PersistMode.kNoPersistParameters;
    public static ResetMode turningReset = ResetMode.kResetSafeParameters;

    public static int drivingMotorCurrentLimit = 80; // amps
    public static int turningMotorCurrentLimit = 20; // amps
}


