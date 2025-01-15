package frc.robot.Climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;

import java.lang.*;

public class climbersubmarine extends TimedRobot{
    public Constants constants = new Constants();
    private static final int deviceID = 1;
    private SparkFlex roboMotorRight;
    private SparkFlex roboMotorLeft;
    private SparkLimitSwitch m_forwardLimit;
    private SparkLimitSwitch m_reverseLimit;

    @Override
    public void robotInit(){
        roboMotorRight = new SparkFlex(deviceID, MotorType.kBrushless);

        roboMotorLeft = new SparkFlex(deviceID, MotorType.kBrushless);
        SparkFlexConfig config = new SparkFlexConfig();

        roboMotorRight.configure(config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        roboMotorLeft.configure(config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    }
    @Override
    public void teleopPeriodic(){
        roboMotorLeft.set(constants.LiftSpeedInitial);
        roboMotorRight.set(constants.LiftSpeedInitial);

        
    }
}