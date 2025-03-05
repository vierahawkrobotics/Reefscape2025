package frc.robot.Climber;

import frc.robot.Climber.ClimberSubmarine;

public class ClimberSetState extends ClimberSubmarine{
    public void setState(climberstate newstate) {
        if (newstate == climberstate.Close){
            System.out.println("Closing");
        } else if (newstate == climberstate.Open) {
            System.out.println("Opening");
        }
        System.out.println("State set");
        ClimberSubmarine.state = newstate;
    }
}
