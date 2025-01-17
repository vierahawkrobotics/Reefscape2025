package frc.robot.Drivetrain;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
public class Drive2D extends Command {
    
    //TO DO: this can be changed later for area effects etc, note it must be meters/second
    double maxSpeed = 4;
    //TO DO: this should be taken from the position subsystem
    double robotAngle = 45.0;
    
    Supplier<Float> vx;
    Supplier<Float> vy;
    Supplier<Float> vr;


    private Drive2D(Supplier<Float> vxInput, Supplier<Float> vyInput) {
        addRequirements(Robot.instance.drivetrain);
        vx = vxInput;
        vy = vyInput;
    }

    @Override
    public void initialize() {}
    @Override
    public void execute() {
        //apply input deadband, input squaring, and scale input by the speed for x, y, and r
        double vxVal = (vx.get() < 0) ? Math.pow(vx.get(),2)*(-1): Math.pow(vx.get(),2);
        vxVal = (vxVal< DrivetrainConstants.inputDeadband)?vxVal=0: vxVal*maxSpeed;

        double vyVal = (vy.get() < 0) ? Math.pow(vy.get(),2)*(-1): Math.pow(vy.get(),2);
        vyVal = (vyVal < DrivetrainConstants.inputDeadband) ? vyVal =0: vyVal*maxSpeed;

        double vrVal = vr.get();
        //set vrVal based on area effects
        // areaPose = Robot.instance.position.AreaEffectHandler.getPose();
        // if (areaPose == null){
        //  vrVal = (vrVal< DrivetrainConstants.inputDeadband)?vrVal=0: vrVal*Robot.instance.drivetrain.rotationSensitivity;
        // }
        // else{vrVal = areaPose.getRotation();}
        
        Robot.instance.drivetrain.setTargetVel(vxVal, vyVal);
        Robot.instance.drivetrain.setTargetPosRot(vrVal);
    }
    @Override
    public void end(boolean interrupted) {}
    @Override
    public boolean isFinished() {
        return false;
    }
}