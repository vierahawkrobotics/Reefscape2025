package frc.robot.Drivetrain;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class DrivetrainConstants {
//-------------------------------------------------------------------SPEEDS-----------------------------------------------------------------------------------
    public static final double defaultDriveSpeed = 1; //default speed that will be used in meters/second
    public static final double defaultRotSpeed = Math.PI; //default speed that will be used for rotation in radians/second
    public static final double maxDriveSpeed = 3.0; // Maximum drive speed in meters/second
    public static final double maxRotSpeed = 2.0; // Maximum rotation speed in radians per second
    public static final double maxDriveAcceleration = 2.0; // Maximum drive acceleration in meters per second squared
    public static final double maxRotAcceleration = 1.0; // Maximum rotation acceleration in radians per second squared
//-----------------------------------------------------------MECHANICAL KNOWLEDGE: Spike-----------------------------------------------------------------------------------
    // //swerve module locations relative to the robot's center in meters
    // public static final Translation2d frontLeftLocation = new Translation2d(0.29845, 0.29845);
    // public static final Translation2d frontRightLocation = new Translation2d(0.29845, -0.29845);
    // public static final Translation2d backLeftLocation = new Translation2d(-0.29845, 0.29845);
    // public static final Translation2d backRightLocation = new Translation2d(-0.29845, -0.29845);
    // public static final Translation2d[] SwerveModulePositions = {frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation};
    // //wheel stuff
    // public static final double wheelDiameterMeters = 0.0762;
    // public static final int drivingMotorPinionTeeth = 5;
    // public static final int drivingMotorSpurTeeth = 50;
    // //IDs
    // public static final int flDrivingID = 1;
    // public static final int flTurningID = 2;
    // public static final int frDrivingID = 3;
    // public static final int frTurningID = 4;
    // public static final int blDrivingID = 5;
    // public static final int blTurningID = 6;
    // public static final int brDrivingID = 7;
    // public static final int brTurningID = 8;


//-----------------------------------------------------------MECHANICAL KNOWLEDGE: Rexy-----------------------------------------------------------------------------------
     //swerve module locations relative to the robot's center in meters
     public static final Translation2d frontLeftLocation = new Translation2d(0.29845, 0.29845);
     public static final Translation2d frontRightLocation = new Translation2d(0.29845, -0.29845);
     public static final Translation2d backLeftLocation = new Translation2d(-0.29845, 0.29845);
     public static final Translation2d backRightLocation = new Translation2d(-0.29845, -0.29845);
     public static final Translation2d[] SwerveModulePositions = {frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation};
     //wheel stuff
     public static final double wheelDiameterMeters = 0.074;
     public static final int drivingMotorPinionTeeth = 14;
     public static final int drivingMotorSpurTeeth = 21;
     //IDs
     public static final int flDrivingID = 3;
     public static final int flTurningID = 7;
     public static final int frDrivingID = 2;
     public static final int frTurningID = 6;
     public static final int blDrivingID = 4;
     public static final int blTurningID = 8;
     public static final int brDrivingID = 1;
     public static final int brTurningID = 5;
  
//-----------------------------------------------------------------PIDS-----------------------------------------------------------------------------------
    //PIDs for the x axis during position control
    public static final double xPositionP = 0.1; 
    public static final double xPositionI = 0.0;  
    public static final double xPositionD = 0.0;  
    //PIDs for the y axis during position control
    public static final double yPositionP = 0.1;  
    public static final double yPositionI = 0.0;  
    public static final double yPositionD = 0.0; 
    //PIDs for rotation during position control
    public static final double rotationP = 0.1;  
    public static final double rotationI = 0.0; 
    public static final double rotationD = 0.0; 
    //PIDs for turning during velocity control
    public static final double turningVelocityP = 0.3;
    public static final double turningVelocityI = 0;
    public static final double turningVelocityD = 0.02;
    //PIDs for driving during velocity control + ff control
    public static final double drivingVelocityP = 0.2;
    public static final double drivingVelocityI = 0;
    public static final double drivingVelocityD = 0.02;
    public static final double drivingVelocityF = 0;

//-----------------------------------------------------STUFF TO TEST/ MESS WITH-----------------------------------------------------------------------------------
    //go to a point stuff
    public static final double atPointTarget = 0.05;
    public static final double atRotTarget = 0.14;
    public static final double stoppedVelocity = 0.02;//velocity considered stopped, will be tested on each individual module
    //current limits
    public static final int drivingCurrentLimit = 80;//in amps
    public static final int turningCurrentLimit = 40;//in amps
    //deadband for joystick control
    public static final double inputDeadband = 0.12;
   
//-----------------------------------------------------INVERSION, VERY IMPORTANT-----------------------------------------------------------------------------------
    public static final boolean invertTurningMotors = false;
    // public static final boolean invertDrivingMotors = false; Use firmware client instead.
    public static final boolean invertTurningEncoders = true;
    // public static final boolean invertDrivingEncoders = false; inversion can't be set for driving encoders

//----------------------------------------------------------------MISC.-----------------------------------------------------------------------------------
    //behavior when the robot is not moving
    public static final IdleMode turningIdleMode = IdleMode.kBrake;
    public static final IdleMode drivingIdleMode = IdleMode.kBrake;
    //calculations for encoders
    public static final double wheelCircumferenceMeters = wheelDiameterMeters * Math.PI;
    public static final double drivingMotorReduction = (45.0 * drivingMotorSpurTeeth) / (drivingMotorPinionTeeth * 15);
    public static final double drivingEncoderPositionFactor = (wheelDiameterMeters * Math.PI) / drivingMotorReduction; // meters
    public static final double drivingEncoderVelocityFactor = drivingEncoderPositionFactor / 60.0; // meters per second
    //Objects that are reused and never change
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        frontLeftLocation, 
        frontRightLocation, 
        backLeftLocation, 
        backRightLocation
    );
    public static final TrapezoidProfile xProfile= new TrapezoidProfile(
        new TrapezoidProfile.Constraints(DrivetrainConstants.maxDriveSpeed, DrivetrainConstants.maxDriveAcceleration)
    );
    public static final TrapezoidProfile yProfile= new TrapezoidProfile(
        new TrapezoidProfile.Constraints(DrivetrainConstants.maxDriveSpeed, DrivetrainConstants.maxDriveAcceleration)
    );
    public static final TrapezoidProfile rotProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(DrivetrainConstants.maxRotSpeed, DrivetrainConstants.maxRotAcceleration)
    );
    public static final SwerveModuleState[] holdSwerveStates = {
            new SwerveModuleState(0.0, new Rotation2d(Math.PI/4)), //fl
            new SwerveModuleState(0.0, new Rotation2d(-Math.PI/4)),//fr
            new SwerveModuleState(0.0, new Rotation2d(-Math.PI/4)),//bl
            new SwerveModuleState(0.0, new Rotation2d(Math.PI/4)) //br
    }; //Swerve states for the holdPosition command.
 
}
