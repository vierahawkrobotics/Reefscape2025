package frc.robot.Climber;


public class ClimberSetState extends climbersubmarine{
    public void setState(climberstate newstate) {
        if (newstate == climberstate.Close){
            System.out.println("Closing");
        } else if (newstate == climberstate.Open) {
            System.out.println("Opening");
        }
        System.out.println("State set");
        state = newstate;
    }
}
