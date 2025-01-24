
package frc.robot.ArmSubsystem;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ElevatorUpDownActionCommand extends Command {

    private SparkMax motor;

    public ElevatorUpDownActionCommand() {
        addRequirements(Robot.instance.exampleSubsystem);
    }

    @Override
    public void initialize() {
        motor = new SparkMax(0, null)
    }

    @Override
    public void execute() {
        if (autoCommand != null) {
            motor.set(1.0);
        }
        else if (autoCommand != null) {
            motor.set(-1.0);
        }
        else {
            motor.set(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        motor.set(0);
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}