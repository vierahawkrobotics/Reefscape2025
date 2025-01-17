package frc.robot.climber;

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
    Close,
    Open,
    Stop,
}

public class climbersubmarine extends SubsystemBase{
    private SparkFlex roborobotLeft;
    private SparkFlex roborobotRight;
    private RelativeEncoder encoder;
    private RelativeEncoder encoder2;
    private double posr;
    private double posl;
    private PIDController pidr = new PIDController(Constants.p, Constants.i, Constants.d);
    private PIDController pidl = new PIDController(Constants.p2, Constants.i2, Constants.d2);
    private climberstate state = climberstate.Open;



    public climbersubmarine(){
        roborobotLeft = new SparkFlex(Constants.MotorIdOne, MotorType.kBrushless);
        roborobotRight = new SparkFlex(Constants.MotorId2, MotorType.kBrushless);
        encoder = roborobotRight.getEncoder();
        encoder2 = roborobotLeft.getEncoder();
    }
    public void periodic() {
        calcPos();
        switch (state) {
            case Close:
                roborobotRight.set(pidr.calculate(posr,Constants.CloseTarget));
                roborobotLeft.set(pidl.calculate(posl,Constants.CloseTarget));
                break;
            case Open:
                if (posr>Constants.OpenLimit && posl>Constants.OpenLimit) {
                    roborobotRight.set(pidr.calculate(posr,Constants.OpenTarget));
                    roborobotLeft.set(pidl.calculate(posl,Constants.OpenTarget));
                } else {
                    state = climberstate.Stop;
                }
                break;

            case Stop:
                roborobotRight.set(0);
                roborobotLeft.set(0);
        }
    }
    public void setState(climberstate newstate) {
        state = newstate;
    }
    private void calcPos(){
        posr = encoder.getPosition()*Constants.rotToRad;
        posl = -encoder2.getPosition()*Constants.rotToRad;
    }
}
