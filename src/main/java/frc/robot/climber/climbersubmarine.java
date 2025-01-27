package frc.robot.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;

enum climberstate {
    Close,
    Open,
}
// defines the possible states of the climber subsystem

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
    // defines motors, pids, positions, and state variables



    public climbersubmarine(){
        System.out.println("Climber submarine initialized");
        roborobotLeft = new SparkFlex(Constants.MotorIdOne, MotorType.kBrushless);
        roborobotRight = new SparkFlex(Constants.MotorId2, MotorType.kBrushless);
        encoder = roborobotRight.getEncoder();
        encoder2 = roborobotLeft.getEncoder();
    }
    //initializing
    public void periodic() {
        System.out.println("Periodic");
        calcPos();
        switch (state) {
            case Close:
                roborobotRight.set(pidr.calculate(posr,Constants.CloseTarget));
                roborobotLeft.set(pidl.calculate(posl,Constants.CloseTarget));
                break;
            case Open:
                if (posr > Constants.OpenLimit) {
                    roborobotRight.set(pidr.calculate(posr,Constants.OpenTarget));
                } else {
                    roborobotRight.set(0);
                }
                if (posl > Constants.OpenLimit) {
                    roborobotLeft.set(pidl.calculate(posl,Constants.OpenTarget));
                } else {
                    roborobotLeft.set(0);
                }
                break;
                
                
        }
    }
    //periodically runs
    public void setState(climberstate newstate) {
        if (newstate == climberstate.Close){
            System.out.println("Closing");
        } else if (newstate == climberstate.Open) {
            System.out.println("Opening");
        }
        System.out.println("State set");
        state = newstate;
    }
    //function to set state of the climber
    private void calcPos(){
        System.out.println("Periodic Position Calculated");
        posr = encoder.getPosition()*Constants.rotToRad;
        posl = -encoder2.getPosition()*Constants.rotToRad;
    }
    //calculates the position of the climber arms
}
