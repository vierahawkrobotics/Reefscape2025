package frc.robot.Drivetrain;

import edu.wpi.first.wpilibj2.command.Command;

public class HoldPosition extends Command {
    /**
     * Calls the holdPosition method in the Drivetrain class which "Sets the PIDs for the driving and turning motor controllers to be 
     * in a "hold position." Here, the wheels form an X which makes it harder to move the bot."
     * @author Giahna C.
     */
    public HoldPosition(){
        addRequirements(Drivetrain.getInstance());
    }
    @Override
    public void initialize() {
        Drivetrain.getInstance().holdPosition();
    }
    @Override
    public void execute() {}
    @Override
    public void end(boolean interrupted) {}
    @Override
    public boolean isFinished() {
        return false;
    }
}
