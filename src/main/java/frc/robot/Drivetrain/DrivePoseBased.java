package frc.robot.Drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class DrivePoseBased extends Command{
    boolean bool;

    public DrivePoseBased(double posX, double posY, double posR, Supplier<Boolean> boolSupplier){
        addRequirements(Robot.instance.drivetrain);
        bool = boolSupplier.get();
        Robot.instance.drivetrain.setTargetPos(posX, posY);
        Robot.instance.drivetrain.setTargetPosRot(posR);
    }
    @Override
    public void execute(){
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        if(bool) return true;
        if(Robot.instance.drivetrain.distance < DrivetrainConstants.validRange && Robot.instance.drivetrain.rotDistance < DrivetrainConstants.validRotDiff){
        return true;
        }
        return false;
    }
}
