package frc.robot.Scratchbot;

import java.util.function.Supplier;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.robot.ComponentsOld.PositionComponent.PositionComponent;
import frc.robot.Drivetrain.GoToPoint;
import frc.robot.Drivetrain.JoystickControl;

public class Scratchbot {
    /**
     * Moves the robot forward by a specified distance in meters.
     * @param metersToMove The distance in meters to move the robot forward.
     */
    public static void sb_moveforward(double metersToMove){
        Pose2d curPose = PositionComponent.getRobotPose();
        Pose2d newPose = new Pose2d(curPose.getX() + metersToMove, curPose.getY(), curPose.getRotation());
        CommandScheduler.getInstance().schedule(new GoToPoint(newPose, true));
    }
    /**
     * Moves the robot left by a specified distance in meters.
     * @param metersToMove The distance in meters to move the robot left.
     */
    public static void sb_moveLeft(double metersToMove){
        Pose2d curPose = PositionComponent.getRobotPose();
        Pose2d newPose = new Pose2d(curPose.getX(), curPose.getY() + metersToMove, curPose.getRotation());
        CommandScheduler.getInstance().schedule(new GoToPoint(newPose, true));
    }
    /**
     * Moves the robot right by a specified distance in meters.
     * @param metersToMove The distance in meters to move the robot right.s
     */
    public static void sb_moveRight(double metersToMove){
        Pose2d curPose = PositionComponent.getRobotPose();
        Pose2d newPose = new Pose2d(curPose.getX(), curPose.getY() - metersToMove, curPose.getRotation());
        CommandScheduler.getInstance().schedule(new GoToPoint(newPose, true));
    }
    /**
     * Moves the robot backward by a specified distance in meters.
     * @param metersToMove The distance in meters to move the robot backward.
     */
    public static void sb_moveBackward(double metersToMove){
        Pose2d curPose = PositionComponent.getRobotPose();
        Pose2d newPose = new Pose2d(curPose.getX() - metersToMove, curPose.getY(), curPose.getRotation());
        CommandScheduler.getInstance().schedule(new GoToPoint(newPose, true));
    }
    /**
     * Turns the robot clockwise by a specified angle in radians.
     * @param radiansToTurn The angle in radians to turn the robot clockwise.
     */
    public static void sb_turnClockwise(double radiansToTurn){
        Pose2d curPose = PositionComponent.getRobotPose();
        Pose2d newPose = new Pose2d(curPose.getX(), curPose.getY(), new Rotation2d(curPose.getRotation().getRadians() - radiansToTurn));
        CommandScheduler.getInstance().schedule(new GoToPoint(newPose, true));
    }
    /**
     * Turns the robot counter-clockwise by a specified angle in radians.
     * @param radiansToTurn The angle in radians to turn the robot counter-clockwise.
     */
    public static void sb_turnCounterClockwise(double radiansToTurn){
        Pose2d curPose = PositionComponent.getRobotPose();
        Pose2d newPose = new Pose2d(curPose.getX(), curPose.getY(), new Rotation2d(curPose.getRotation().getRadians() + radiansToTurn));
        CommandScheduler.getInstance().schedule(new GoToPoint(newPose, true));
    }
    /**
     * @return The value of the left joystick's X axis of the controller.
     */
    public static double sb_getControllerLeftX(){
        return Robot.controller.getLeftX();
    }
    /**
     * @return The value of the left joystick's Y axis of the controller.
     */
    public static double sb_getControllerLeftY(){
        return Robot.controller.getLeftY();
    }
    /**
     * @return The value of the right joystick's X axis of the controller.
     */
    public static double sb_getControllerRightX(){
        return Robot.controller.getRightX();
    }
    /**
     * @return The value of the right joystick's Y axis of the controller.
     */
    public static double sb_getControllerRightY(){
        return Robot.controller.getRightY();
    }
    /**
     * Sets the robot's movement using joystick control (or any other suppliers). 
     * @param xVelocity Supplier for the X velocity of the robot. [-1,1]
     * @param yVelocity Supplier for the Y velocity of the robot. [-1,1]
     */
    public static void sb_setRobotMovement(Supplier<Double> xVelocity, Supplier<Double> yVelocity){
        CommandScheduler.getInstance().schedule(new JoystickControl(xVelocity,yVelocity, () -> {return 0.0;}, () -> {return 0.0;}));
    }
    /**
     * Creates a configured CANdle object with default settings.
     * @return the configured CANdle object.
     */
    public static CANdle sb_makeConfiguredCandle(){
        CANdle candle = new CANdle(20);

        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB;
        config.brightnessScalar = 1.0;
        candle.configAllSettings(config, 100);

        candle.setLEDs(0, 0, 0);

        return candle;
    }
}
