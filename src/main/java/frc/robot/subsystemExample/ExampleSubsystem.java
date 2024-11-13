package frc.robot.SubsystemExample;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {
    private ShuffleboardTab tab;
    public ExampleSubsystem() {
        tab = Shuffleboard.getTab("example subsystem");
    }

    @Override
    public void periodic() {}
    @Override
    public void simulationPeriodic() {}
}
