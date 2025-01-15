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
    NormalOper
}

public class ArmSubsystem extends SubsystemBase {
    private ShuffleboardTab armTab;
    private ArmState state = ArmState.NormalOper;
    private SparkFlex elevator;
    private SparkFlex elevatorFollower;
    private double targetHeight;
    private double curHeight = 0;
    PIDController elevatorPID = new PIDController(ArmConstants.p, ArmConstants.i, ArmConstants.d);
    
    public ArmSubsystem() {
        armTab = Shuffleboard.getTab("Arm Subsystem");

        elevator = new SparkFlex(ArmConstants.elevatorMotorID,MotorType.kBrushless);
        elevatorFollower = new SparkFlex(ArmConstants.elevatorFollowMotorID,MotorType.kBrushless);
        elevator.getExternalEncoder().setPosition(0);
        SparkBaseConfig elevatorFollowerConfig = new SparkFlexConfig();
        elevatorFollowerConfig.follow(elevator);
        elevatorFollower.configure(elevatorFollowerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    }

    private void setTargetHeight(double newTargetHeight) {
        targetHeight = newTargetHeight;
    }

    double getHeight() {
        return curHeight;
    }

    boolean atTargetHeight() {
        if (targetHeight == elevator.getExternalEncoder().getPosition()) {
            return true;
        } else {
            return false;
        }
    }

    private void RecallibrateHeight() {
        elevator.set(ArmConstants.neutralMotorBias + ArmConstants.resetHeightModeBias);
        if(atTargetHeight()) {
            elevator.getExternalEncoder().setPosition(0);
            state = ArmState.NormalOper;
        }
    }
    
    @Override
    public void periodic() {
        curHeight = elevator.getExternalEncoder().getPosition()*2*Math.PI*ArmConstants.radius;
        setTargetHeight(targetHeight);
        switch (state) {
            case ResetHeight:
                RecallibrateHeight();
                break;
            case NormalOper:
                elevator.set(elevatorPID.calculate(getHeight(),targetHeight));
                break;
        }
    }
    @Override
    public void simulationPeriodic() {}
}
