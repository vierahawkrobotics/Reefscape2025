package frc.robot.climber;

import java.lang.Math;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Constants {
    public  double LiftSpeedInitial = 10;
    public static double OpenLimit = 0.1;
    public static double rotToRad = 1;
    public static double LiftSpeed = 12;    
    public static double LiftSpeedEnd = 0;    
    public static double OpenTarget = Math.PI/2;
    public static double CloseTarget = 0;    
    public static int MotorIdOne = -1;
    public static int MotorId2 = -1;
    public static XboxController controller = new XboxController(1);
    public EventLoop eventLoop = new EventLoop();
    public Trigger climberButton = new Trigger(controller.leftBumper(eventLoop));
    final public static double p = 1;
    final public static  double i = 0;
    final public static double d = 0;
    final public static double p2 = 1;
    final public static double i2 = 0;
    final public static double d2 = 0;
}

