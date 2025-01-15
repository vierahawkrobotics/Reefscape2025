package frc.robot.Climber;

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

public class climbersub extends TimedRobot{
    private Joystick m_stick;
    private static final int deviceID = 1;
    private SparkLowLevel m_motor;
    private SparkLimitSwitch m_forwardLimit;
    private SparkLimitSwitch m_reverseLimit;
  
    public String kEnable;
    public String kDisable;
  
    @Override
    public void robotInit() {
      // initialize SPARK MAX with CAN ID
      m_motor = new SparkFlex(deviceID, MotorType.kBrushless);
      SparkFlexConfig config = new SparkFlexConfig();
      /**
       * The RestoreFactoryDefaults method can be used to reset the configuration parameters
       * in the SPARK MAX to their factory default state. If no argument is passed, these
       * parameters will not persist between power cycles
       */
      m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
      /**
       * A SparkLimitSwitch object is constructed using the getForwardLimitSwitch() or
       * getReverseLimitSwitch() method on an existing CANSparkMax object, depending
       * on which direction you would like to limit
       * 
       * Limit switches can be configured to one of two polarities:
       *  com.revrobotics.SparkLimitSwitch.SparkLimitSwitch.Type.kNormallyOpen
       *  com.revrobotics.SparkLimitSwitch.SparkLimitSwitch.Type.kNormallyClosed
       */
      m_forwardLimit = m_motor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
      m_reverseLimit = m_motor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
  
      m_stick = new Joystick(0);
  
      /**
       * Limit switches are enabled by default when they are intialized. They can be disabled
       * by calling enableLimitSwitch(false) on a SparkLimitSwitch object
       * 
       * Limit switches can be reenabled by calling enableLimitSwitch(true)
       * 
       * The isLimitSwitchEnabled() method can be used to check if the limit switch is enabled
       */
      m_forwardLimit.enableLimitSwitch(false);
      m_reverseLimit.enableLimitSwitch(false);
      SmartDashboard.putBoolean("Forward Limit Enabled", m_forwardLimit.isLimitSwitchEnabled());
      SmartDashboard.putBoolean("Reverse Limit Enabled", m_reverseLimit.isLimitSwitchEnabled());
    }
  
    @Override
    public void teleopPeriodic() {
      m_motor.set(m_stick.getY());
  
      // enable/disable limit switches based on value read from SmartDashboard
      m_forwardLimit.enableLimitSwitch(SmartDashboard.getBoolean("Forward Limit Enabled", false));
      m_reverseLimit.enableLimitSwitch(SmartDashboard.getBoolean("Reverse Limit Enabled", false));
  
      /**
       * The isPressed() method can be used on a SparkLimitSwitch object to read the state of the switch.
       * 
       * In this example, the polarity of the switches eiasjdjkdaskqweqeehudsandsajkwqm,qwewhocdm eorq3nfew89ew cv98ewkjfdoieljicv opwfeeoidsvkoewmlkds8eweioc980ewuidv-sgghoi3w3joeiwenklelkglkno8121oihoihsaoihdqiwduhdohqwoidhqowiheoqiwjdoiqsodoiwhdoihqhwqdoihqwihiqowhoiwqhoqidoiqwhodqhwoidhhoqwihodiqhwodihoihqoidoihwqiooijiojoijidqoijioqdjijdqj8udshuiaeghyujkaemskfrtgyunhjk are set to normally closed. In this case,
       * isPressed() will return true if the switch is pressed. It will also return true if you do not 
       * have a switch connected. isPressed() will return false when the switch is released.
       */
      SmartDashboard.putBoolean("Forward Limit Switch", m_forwardLimit.isPressed());
      SmartDashboard.putBoolean("Reverse Limit Switch", m_reverseLimit.isPressed());
    }
}