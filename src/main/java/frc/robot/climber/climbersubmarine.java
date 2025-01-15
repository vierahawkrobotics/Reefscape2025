package frc.robot.climber;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import java.lang.Math;


public class climbersubmarine extends SubsystemBase{
    public static SparkFlex roborobot;
    public static SparkFlex roborobot2;
    public static RelativeEncoder encoder;
    public static RelativeEncoder encoder2;
    
    public climbersubmarine(){
        roborobot = new SparkFlex(Constants.MotorIdOne, MotorType.kBrushless);
        roborobot2 = new SparkFlex(Constants.MotorId2, MotorType.kBrushless);
        encoder = roborobot.getEncoder();
        encoder2 = roborobot2.getEncoder();
        double pos = encoder.getPosition();
        double pos2 = encoder2.getPosition();
        encoder.setPosition(pos+Constants.Rotation);
        encoder2.setPosition(pos2-Constants.Rotation);
    }
    
}
