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
import edu.wpi.first.math.controller.PIDController;
import java.lang.Math;

enum climberstate {
    ResetPos,
    NormalOper,
    stopped
}

public class climbersubmarine extends SubsystemBase{
    public static SparkFlex roborobot;
    public static SparkFlex roborobot2;
    public static RelativeEncoder encoder;
    public static RelativeEncoder encoder2;
    public static double pos;
    public static double pos2;
    public static double posmove;
    public static double posmove2;
    public static PIDController pid1 = new PIDController(Constants.p, Constants.i, Constants.d);
    public static PIDController pid2 = new PIDController(Constants.p, Constants.i, Constants.d);
    public climberstate state = climberstate.NormalOper;



    public climbersubmarine(){
        roborobot = new SparkFlex(Constants.MotorIdOne, MotorType.kBrushless);
        roborobot2 = new SparkFlex(Constants.MotorId2, MotorType.kBrushless);
        encoder = roborobot.getEncoder();
        encoder2 = roborobot2.getEncoder();
        
    }
    public void periodic() {
        switch (state) {
            case ResetPos:
                roborobot.set(pid1.calculate(pos*2*Math.PI,0));
                roborobot2.set(pid2.calculate(pos2*2*Math.PI,0));
                break;
            case NormalOper:
                go();
                roborobot.set(pid1.calculate(pos*2*Math.PI,Constants.Rotation));
                roborobot2.set(pid2.calculate(pos2*2*Math.PI,Constants.Rotation));
                break;
            case stopped:
                roborobot.set(0);
                roborobot2.set(0);
        }
    }
    public void initgo(){
        
    }
    public void go(){
        pos = encoder.getPosition();
        pos2 = encoder2.getPosition();
        //posmove = pid1.calculate(pos*2*Math.PI,targetHeight)
        //encoder.setPosition(Constants.Rotation);
        //encoder2.setPosition(Constants.Rotation);
    }
    public void waitnvm(){
        //encoder.setPosition(pos);
        //encoder2.setPosition(pos2);
    }
}
