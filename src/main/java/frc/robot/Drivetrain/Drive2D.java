package frc.robot.Drivetrain;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
public class Drive2D extends Command {
    
    //TO DO: this should be taken from the position subsystem
    double robotAngle = 45.0;
    
    Supplier<Double> vx;
    Supplier<Double> vy;
    Supplier<Double> vr;


    private Drive2D(Supplier<Double> vxInput, Supplier<Double> vyInput, Supplier<Double> vrInput) {
        addRequirements(Robot.instance.drivetrain);
        vx = vxInput;
        vy = vyInput;
        vr = vrInput;
    }

    @Override
    public void initialize() {}
    @Override
    public void execute() {
        //apply input deadband, input squaring, and scale input by the speed for x, y, and r
        double vxVal = (vx.get() < 0) ? Math.pow(vx.get(),2)*(-1): Math.pow(vx.get(),2);
        vxVal = (vxVal< DrivetrainConstants.inputDeadband)?vxVal=0: vxVal;

        double vyVal = (vy.get() < 0) ? Math.pow(vy.get(),2)*(-1): Math.pow(vy.get(),2);
        vyVal = (vyVal < DrivetrainConstants.inputDeadband) ? vyVal =0: vyVal;

        double vrVal;
        //set vrVal based on area effects
        // areaPose = Robot.instance.position.AreaEffectHandler.getPose();
        // if (areaPose == null){
        //  vrVal = (vr.get()< DrivetrainConstants.inputDeadband)?0: vr.get();
        //  Robot.instance.drivetrain.setTargetVelRot(vrVal);
        // }
        // else{
        //     vrVal = areaPose.getRotation();
        //     Robot.instance.drivetrain.setTargetPosRot(vrVal);
        // }
        
        Robot.instance.drivetrain.setTargetVel(vxVal, vyVal);
    }
    @Override
    public void end(boolean interrupted) {}
    @Override
    public boolean isFinished() {
        return false;
    }
}