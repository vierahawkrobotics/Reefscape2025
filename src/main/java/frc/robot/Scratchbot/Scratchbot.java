package frc.robot.Scratchbot;

import java.util.function.Supplier;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.robot.Components.PositionComponent.PositionComponent;
import frc.robot.Drivetrain.GoToPoint;
import frc.robot.Drivetrain.JoystickControl;

public class Scratchbot {
    public static void sb_moveforward(double metersToMove){
        Pose2d curPose = PositionComponent.getRobotPose();
        Pose2d newPose = new Pose2d(curPose.getX() + metersToMove, curPose.getY(), curPose.getRotation());
        CommandScheduler.getInstance().schedule(new GoToPoint(newPose, true));
    }
    public static void sb_moveLeft(double metersToMove){
        Pose2d curPose = PositionComponent.getRobotPose();
        Pose2d newPose = new Pose2d(curPose.getX(), curPose.getY() + metersToMove, curPose.getRotation());
        CommandScheduler.getInstance().schedule(new GoToPoint(newPose, true));
    }
    public static void sb_moveRight(double metersToMove){
        Pose2d curPose = PositionComponent.getRobotPose();
        Pose2d newPose = new Pose2d(curPose.getX(), curPose.getY() - metersToMove, curPose.getRotation());
        CommandScheduler.getInstance().schedule(new GoToPoint(newPose, true));
    }
    public static void sb_moveBackward(double metersToMove){
        Pose2d curPose = PositionComponent.getRobotPose();
        Pose2d newPose = new Pose2d(curPose.getX() - metersToMove, curPose.getY(), curPose.getRotation());
        CommandScheduler.getInstance().schedule(new GoToPoint(newPose, true));
    }
    public static void sb_turnClockwise(double radiansToTurn){
        Pose2d curPose = PositionComponent.getRobotPose();
        Pose2d newPose = new Pose2d(curPose.getX(), curPose.getY(), new Rotation2d(curPose.getRotation().getRadians() - radiansToTurn));
        CommandScheduler.getInstance().schedule(new GoToPoint(newPose, true));
    }
    public static void sb_turnCounterClockwise(double radiansToTurn){
        Pose2d curPose = PositionComponent.getRobotPose();
        Pose2d newPose = new Pose2d(curPose.getX(), curPose.getY(), new Rotation2d(curPose.getRotation().getRadians() + radiansToTurn));
        CommandScheduler.getInstance().schedule(new GoToPoint(newPose, true));
    }
    public static double sb_getControllerLeftX(){
        return Robot.controller.getLeftX();
    }
    public static double sb_getControllerLeftY(){
        return Robot.controller.getLeftY();
    }
    public static double sb_getControllerRightX(){
        return Robot.controller.getRightX();
    }
    public static double sb_getControllerRightY(){
        return Robot.controller.getRightY();
    }
    public static void sb_setRobotMovement(Supplier<Double> xVelocity, Supplier<Double> yVelocity){
        CommandScheduler.getInstance().schedule(new JoystickControl(xVelocity,yVelocity, () -> {return 0.0;}, () -> {return 0.0;}));
    }
    // public static CANdle sb_makeConfiguredCandle(){
    //     candle = new CANdle(0);
    // }
}
