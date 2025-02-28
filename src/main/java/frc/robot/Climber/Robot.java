package frc.robot.Climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.security.spec.EncodedKeySpec;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Robot extends TimedRobot {
    private static final int deviceID = 1;
    private SparkMax m_motor    ;
    private PIDController m_pidController;
    private Encoder m_encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
  
    @Override
    public void robotInit() {
      // initialize motor
      m_motor = new SparkMax(deviceID, MotorType .kBrushless);
  
      /**
       * The RestoreFactoryDefaults method can be used to reset the configuration parameters
       * in the SPARK MAX to their factory default state. If no argument is passed, these
       * parameters will not persist between power cycles
       */  
      // initialze PID controller and encoder objects
      m_pidController = m_motor.();
      m_encoder = m_motor.getEncoder();
  
      // PID coefficients
      kP = 5e-5; 
      kI = 1e-6;
      kD = 0; 
      kIz = 0; 
      kFF = 0.000156; 
      kMaxOutput = 1; 
      kMinOutput = -1;
      maxRPM = 5700;
  
      // Smart Motion Coefficients
      maxVel = 2000; // rpm
      maxAcc = 1500;
  
      // set PID coefficients
      m_pidController.setP(kP);
      m_pidController.setI(kI);
      m_pidController.setD(kD);
      m_pidController.setIZone(kIz);
      m_pidController.setFF(kFF);
      m_pidController.setOutputRange(kMinOutput, kMaxOutput);
  
      /**
       * Smart Motion coefficients are set on a SparkPIDController object
       * 
       * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
       * the pid controller in Smart Motion mode
       * - setSmartMotionMinOutputVelocity() will put a lower bound in
       * RPM of the pid controller in Smart Motion mode
       * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
       * of the pid controller in Smart Motion mode
       * - setSmartMotionAllowedClosedLoopError() will set the max allowed
       * error for the pid controller in Smart Motion mode
       */
      int smartMotionSlot = 0;
      m_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
      m_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
      m_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
      m_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
  
      // display PID coefficients on SmartDashboard
      SmartDashboard.putNumber("P Gain", kP);
      SmartDashboard.putNumber("I Gain", kI);
      SmartDashboard.putNumber("D Gain", kD);
      SmartDashboard.putNumber("I Zone", kIz);
      SmartDashboard.putNumber("Feed Forward", kFF);
      SmartDashboard.putNumber("Max Output", kMaxOutput);
      SmartDashboard.putNumber("Min Output", kMinOutput);
  
      // display Smart Motion coefficients
      SmartDashboard.putNumber("Max Velocity", maxVel);
      SmartDashboard.putNumber("Min Velocity", minVel);
      SmartDashboard.putNumber("Max Acceleration", maxAcc);
      SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
      SmartDashboard.putNumber("Set Position", 0);
      SmartDashboard.putNumber("Set Velocity", 0);
  
      // button to toggle between velocity and smart motion modes
      SmartDashboard.putBoolean("Mode", true);
    }
  
    @Override
    public void teleopPeriodic() {
        
  }
}