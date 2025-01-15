package frc.robot.Climber;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import java.lang.Math;

public class climbersubmarine extends Robot {
    public static SparkFlex roborobot;
    public static SparkFlex roborobot2;
    public static RelativeEncoder encoder;
    public static RelativeEncoder encoder2;

    public Void climbersubmarines() {
        double pos = encoder.getPosition();
        double pos2 = encoder2.getPosition();
        encoder.setPosition(pos + Constants.Rotation);
        encoder2.setPosition(pos2 - Constants.Rotation);

    }

    public Constants constants = new Constants();
    private static final int deviceID = 1;
    private SparkFlex roboMotorRight;
    private SparkFlex roboMotorLeft;
    private SparkLimitSwitch m_forwardLimit;
    private SparkLimitSwitch m_reverseLimit;

    public void execute(){

    }
    @Override
    public void teleopInit() {
        roborobot = new SparkFlex(Constants.MotorIdOne, MotorType.kBrushless);
        roborobot2 = new SparkFlex(Constants.MotorId2, MotorType.kBrushless);
        encoder = roborobot.getEncoder();
        encoder2 = roborobot2.getEncoder();
    }

}
