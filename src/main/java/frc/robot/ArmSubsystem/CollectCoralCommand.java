package frc.robot.ArmSubsystem;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
public class CollectCoralCommand extends Command {
    
    private SparkMax motor;

    public CollectCoralCommand() {
        addRequirements(Robot.instance.exampleSubsystem);
    }

    @Override
    public void initialize() {
        motor = new SparkMax(0, null);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
