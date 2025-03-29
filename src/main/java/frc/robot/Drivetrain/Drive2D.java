package frc.robot.Drivetrain;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Components.AreaEffects.AreaEffectsHandler;
import frc.robot.Components.PositionTools.PositionTools;
public class Drive2D extends Command {
    
    //TO DO: this should be taken from the position subsystem
    double robotAngle = 45.0;
    
    Supplier<Double> vx;
    Supplier<Double> vy;
    Supplier<Double> vr;
    Supplier<Boolean> doAlign;


    public Drive2D(Supplier<Double> vxInput, Supplier<Double> vyInput, Supplier<Double> vrInput, Supplier<Boolean> doAlign) {
        addRequirements(Robot.instance.drivetrain);
        vx = vxInput;
        vy = vyInput;
        vr = vrInput;
        this.doAlign = doAlign;
    }

    @Override
    public void initialize() {}
    @Override
    public void execute() {
        //apply input deadband, input squaring, and scale input by the speed for x, y, and r
        double vxVal = vx.get();
        double vyVal = vy.get();
        double m = Math.sqrt(vxVal*vxVal+vyVal*vyVal);
    
        if(m <= DrivetrainConstants.inputDeadband) {
            vxVal = 0;
            vyVal = 0;
        }
        else {
            vxVal *= m;
            vyVal *= m;
        }

        // set vrVal based on area effects
        Pose2d areaPose = AreaEffectsHandler.getTargetPose();
        Boolean autoAlign = AreaEffectsHandler.getIsAutoAlign();
        if(doAlign.get() && autoAlign != null && autoAlign.booleanValue()) {
            double vrVal;
            vrVal = PositionTools.closestScorePoseEntry(false).getRotation().getRadians();
            Robot.instance.drivetrain.setTargetPosRot(vrVal);
        }
        else if (doAlign.get() && areaPose != null){
            double vrVal;
            vrVal = areaPose.getRotation().getRadians();
            Robot.instance.drivetrain.setTargetPosRot(vrVal);
        }
        else{
            Double vrVal;
            vrVal = vr.get();
            if(vrVal != null) Robot.instance.drivetrain.setTargetPosRot(vrVal);
        }
        
        Robot.instance.drivetrain.setInputVel(vxVal, vyVal);
    }
    @Override
    public void end(boolean interrupted) {}
    @Override
    public boolean isFinished() {
        return false;
    }
}