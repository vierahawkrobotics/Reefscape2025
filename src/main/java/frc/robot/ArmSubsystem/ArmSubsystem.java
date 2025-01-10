package frc.robot.ArmSubsystem;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

enum ArmState {
    ResetHeight,
    NormalOper,
}

public class ArmSubsystem extends SubsystemBase {
    private ShuffleboardTab tab;
    private SparkFlex elevator;
    private SparkFlex elevatorFollower;
    private float targetHeight;
    PIDController elevatorPID = new PIDController(ArmConstants.p, ArmConstants.i, ArmConstants.d);
    
    public ArmSubsystem() {
        tab = Shuffleboard.getTab("example subsystem");

        elevator = new SparkFlex(ArmConstants.elevatorMotorID,MotorType.kBrushless);
        elevatorFollower = new SparkFlex(ArmConstants.elevatorFollowMotorID,MotorType.kBrushless);
        elevator.getEncoder().setPosition(0);
        SparkBaseConfig elevatorFollowerConfig = new SparkFlexConfig();
        elevatorFollowerConfig.follow(elevator);
        elevatorFollower.configure(elevatorFollowerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    }

    //setTargetHeight(float targetHeight) set targetHeight variable
    //have constraints to prevent illegal input

    //float getHeight()

    //RecallibrateHeight()
    @Override
    public void periodic() {

        //reset height state:
        //  move arm slowly down until limit switch hit
        //  elevator.getEncoder().setPosition(0);
        //  set state to normal oper
        //normal oper state
        elevator.set(elevatorPID.calculate(target, current));

    }
    @Override
    public void simulationPeriodic() {}
}
