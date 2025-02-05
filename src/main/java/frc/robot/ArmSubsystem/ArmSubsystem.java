package frc.robot.ArmSubsystem;

import java.util.Queue;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private ShuffleboardTab armTab;
    private ArmState state = ArmState.NormalOper;
    private static IntakeState intakeState = IntakeState.Rest;
    private SparkFlex elevator;
    private SparkFlex elevatorFollower;
    public static SparkFlex container;
    public static SparkFlex containerFollower;
    private double targetHeight;
    private double curHeight = 0;
    PIDController elevatorPID = new PIDController(ArmConstants.p, ArmConstants.i, ArmConstants.d);
    private double limitSwitchOffset;

    public ArmSubsystem() {
        armTab = Shuffleboard.getTab("Arm Subsystem");

        // Create and setup motors for Elevator
        elevator = new SparkFlex(ArmConstants.elevatorMotorID, MotorType.kBrushless);
        elevatorFollower = new SparkFlex(ArmConstants.elevatorFollowMotorID, MotorType.kBrushless);
        elevator.getExternalEncoder().setPosition(0);
        SparkBaseConfig elevatorFollowerConfig = new SparkFlexConfig();
        elevatorFollowerConfig.follow(elevator);
        elevatorFollower.configure(elevatorFollowerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        // Create and setup motors for Drop and Collect
        container = new SparkFlex(ArmConstants.containerMotorID, MotorType.kBrushless);
        containerFollower = new SparkFlex(ArmConstants.containerFollowMotorID, MotorType.kBrushless);
        container.getExternalEncoder().setPosition(0);
        SparkBaseConfig containerFollowerConfig = new SparkFlexConfig();
        containerFollowerConfig.follow(container);
        containerFollowerConfig.inverted(true);
        containerFollower.configure(containerFollowerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    }

    public void ejectAlgae(){
        SetTargetHeight(GetHeight()-ArmConstants.algaeOffset);
    }

    public double isLimitSwitchPressed() { // return number based on limit switch pressed
        if (container.getForwardLimitSwitch().isPressed()) { // far left channel pressed
            return ArmConstants.farLeftIntakeChannel;
        } else if (container.getReverseLimitSwitch().isPressed()) { // middle left channel pressed
            return ArmConstants.middleLeftIntakeChannel;
        } else if (containerFollower.getForwardLimitSwitch().isPressed()) { // middle right channel pressed
            return ArmConstants.middleRightIntakeChannel;
        } else if (containerFollower.getReverseLimitSwitch().isPressed()) { // far right channel pressed
            return ArmConstants.farRightIntakeChannel;
        } else { // none pressed
            return 0;
        }
    }

    public static void setIntakeState(IntakeState state) { // Change current intake state
        intakeState = state;
    }

    public static IntakeState getIntakeState() { // Return current intake state
        return intakeState;
    }

    public void SetTargetHeight(double targetHeight) { // Set target height
        targetHeight = Math.min(Math.max(targetHeight,ArmConstants.armHeight),ArmConstants.maxHeight);
    }

    public double GetHeight() { // Get current height
        return curHeight + ArmConstants.armHeight;
    }

    public boolean AtTargetHeight() { // Check if at target height
        if ((targetHeight - elevator.getExternalEncoder().getPosition()) < ArmConstants.epsilon) {
            return true;
        } else {
            return false;
        }
    }

    public void RecallibrateHeight() {
        elevator.set(ArmConstants.neutralMotorBias + ArmConstants.resetHeightModeBias);
        if(elevator.getForwardLimitSwitch().isPressed()) {
            elevator.getExternalEncoder().setPosition(0);
            state = ArmState.NormalOper;
        }
    }
    
    @Override
    public void periodic() {
        switch (intakeState) {
            case Rest:
                container.set(0);
                break;
            case Collect:
                container.set(ArmConstants.containerMotorSpeed);
                limitSwitchOffset = isLimitSwitchPressed();
                if (limitSwitchOffset != 0) {
                    intakeState = IntakeState.Rest;
                }
                break;
            case Drop:
                container.set(-ArmConstants.containerMotorSpeed);
                if (Timer.getFPGATimestamp()-DropCoralCommand.startTime == ArmConstants.dropTime) {
                    intakeState = IntakeState.Rest;
                }
                break;
        }

        curHeight = elevator.getExternalEncoder().getPosition()*2*Math.PI*ArmConstants.gearRadius;
        switch (state) {
            case ResetHeight:
                RecallibrateHeight();
                break;
            case NormalOper:
                elevator.set(elevatorPID.calculate(GetHeight(),targetHeight)+ArmConstants.pidCoefficient);
                break;
        }
    }
    @Override
    public void simulationPeriodic() {}

    public enum IntakeState {
        Rest,
        Collect,
        Drop,
    }

    enum ArmState {
        ResetHeight,
        NormalOper
    }
}
 