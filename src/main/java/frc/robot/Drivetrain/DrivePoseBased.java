package frc.robot.Drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class DrivePoseBased extends Command{
    Supplier<Double> posX;
    Supplier<Double> posY;
    Supplier<Double> posR;

    private DrivePoseBased(Supplier<Double> posXInput, Supplier<Double> posYInput, Supplier<Double> posRInput){
        posX = posXInput;
        posY = posYInput;
        posR = posRInput;
        addRequirements(Robot.instance.drivetrain);
    }
    @Override
    public void execute(){
        Robot.instance.drivetrain.setTargetPos(posX.get(), posY.get());
        Robot.instance.drivetrain.setTargetPosRot(posR.get());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
