package frc.robot.Drivetrain;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;

public class JoystickControl extends Command{
    
    Supplier<Double> vx;
    Supplier<Double> vy;
    Supplier<Double> vr;
    double velX;
    double velY;
    double velR;

    public JoystickControl(Supplier<Double> vx, Supplier<Double> vy, Supplier<Double> vr){
        this.vx = vx;
        this.vy = vy;
        this.vr = vr;
        addRequirements(Drivetrain.getInstance());
    }
    @Override
    public void execute(){
        //input squaring
        //input deadband
        if (MiscMathFunctions.distance(0, vx.get(), 0, vy.get()) < DrivetrainConstants.inputDeadband){
            velX = 0;
            velY = 0;
        }
        // Drivetrain.getInstance().setVelocityPIDs(vx.get(), vy.get(), vr.get(), true);
    }
}
