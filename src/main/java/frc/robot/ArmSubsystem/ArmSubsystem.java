package frc.robot.ArmSubsystem;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

enum ArmState {
    ResetHeight,
    NormalOper
}

public class ArmSubsystem extends SubsystemBase {
    private ShuffleboardTab armTab;
    private ArmState state = ArmState.ResetHeight; // Immediately reset height
    private ArmConstants.IntakeState intakeState = ArmConstants.IntakeState.Rest;
    private SparkFlex elevator;
    private SparkFlex elevatorFollower;
    public SparkFlex container;
    public SparkFlex containerFollower;
    private double targetHeight = ArmConstants.minHeight;
    private double curHeight = 0;
    PIDController elevatorPID = new PIDController(ArmConstants.elevatorP, ArmConstants.elevatorI, ArmConstants.elevatorD);
    public double limitSwitchOffset;
    public double startTime;

    public ArmSubsystem() {
        // Shuffleboard Setup
        armTab = Shuffleboard.getTab("Arm Subsystem");
        armTab.addNumber("Current Height", () -> {return curHeight;});
        armTab.addNumber("Current Height Inches", () -> {return curHeight*39.37;});
        armTab.addDouble("Elevator Height Radians", () -> {return elevatorFollower.getExternalEncoder().getPosition();});
        armTab.addNumber("Target Height", () -> {return targetHeight;});
        armTab.addString("Collector State", () -> {return intakeState.toString();});
        armTab.addBoolean("Limit Switch", () -> {return elevator.getReverseLimitSwitch().isPressed();});
        
        // Elevator Motors Setup
        elevator = new SparkFlex(ArmConstants.elevatorMotorID, MotorType.kBrushless);
        elevatorFollower = new SparkFlex(ArmConstants.elevatorFollowMotorID, MotorType.kBrushless);
        SparkFlexConfig elevatorConfig = new SparkFlexConfig();
        elevatorConfig.idleMode(IdleMode.kBrake);
        elevatorConfig.limitSwitch
            .reverseLimitSwitchEnabled(false);
        elevator.configure(elevatorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        SparkFlexConfig elevatorFollowerConfig = new SparkFlexConfig();
        elevatorFollowerConfig.externalEncoder
            .measurementPeriod(50)
            .countsPerRevolution(8192)
            .positionConversionFactor(ArmConstants.encoderPositionFactor)
            .velocityConversionFactor(ArmConstants.encoderVelocityFactor);
        elevatorFollowerConfig
            .follow(elevator, true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ArmConstants.maxAmp);
        elevatorFollower.configure(elevatorFollowerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        // Container Motors Setup
        container = new SparkFlex(ArmConstants.containerMotorID, MotorType.kBrushless);
        containerFollower = new SparkFlex(ArmConstants.containerFollowMotorID, MotorType.kBrushless);
        SparkFlexConfig containerConfig = new SparkFlexConfig();
        containerConfig.limitSwitch
            .forwardLimitSwitchEnabled(false)
            .reverseLimitSwitchEnabled(false);
        containerConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ArmConstants.maxAmp);
        container.configure(containerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        SparkFlexConfig containerFollowerConfig = new SparkFlexConfig();
        containerFollowerConfig.limitSwitch
            .forwardLimitSwitchEnabled(false)
            .reverseLimitSwitchEnabled(false);
        containerFollowerConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ArmConstants.maxAmp);
        containerFollower.configure(containerFollowerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    }
    
    /**
     * Move elevator arm to height registered with HeightState
     * @param height target HeightState
     * @author Andrew S
     */
    public void setHeightState(ArmConstants.HeightState height) {
        SetTargetHeight(height.getHeight());
    }

    /**
     * Change current IntakeState and start timer if changing to Drop state
     * @param state new intake state
     * @author Andrew S
     */
    public void setIntakeState(ArmConstants.IntakeState state) {
        if (state == ArmConstants.IntakeState.Drop && intakeState != ArmConstants.IntakeState.Drop) {
            startTime = Timer.getFPGATimestamp();
        }
        intakeState = state;
    }

    /**
     * @return current intake state
     * @author Andrew S
     */
    public ArmConstants.IntakeState getIntakeState() {
        return intakeState;
    }

    /**
     * Set target height based on the smallest between targetHeight and max armHeight
     * @param targetHeight target arm height
     * @author Andrew S
     */
    public void SetTargetHeight(double targetHeight) {
        this.targetHeight = Math.min(Math.max(targetHeight,ArmConstants.minHeight),ArmConstants.maxHeight);
    }

    /**
     * @return current height
     * @author Andrew S
     */
    public double getHeight() {
        return curHeight;
    }
    /**
     * @return current target height
     * @author Andrew S
     */
    public double getTargetHeight() {
        return targetHeight;
    }

    /**
     * @return if at target height
     * @author Andrew S
     */
    public boolean AtTargetHeight() {
        return Math.abs(targetHeight - getHeight()) < ArmConstants.epsilon;
    }

    /**
     * Reset height to 0
     * @author Andrew S
     */
    private void RecallibrateHeight() {
        if(!elevator.getReverseLimitSwitch().isPressed()) {
            elevatorFollower.getExternalEncoder().setPosition(0);
            state = ArmState.NormalOper;
            elevator.set(0);
        }
        else {
            elevator.set(ArmConstants.resetHeightModeBias);
        }
    }
    
    @Override
    public void periodic() {
        curHeight = elevatorFollower.getExternalEncoder().getPosition()*ArmConstants.gearRadius + ArmConstants.minHeight;
        
        switch (intakeState) { // Collect and Drop
            case Rest:
                container.set(0);
                containerFollower.set(0);
                break;
            case Drop:
                if (Timer.getFPGATimestamp()-startTime >= ArmConstants.containerDropTime) {
                    intakeState = ArmConstants.IntakeState.Rest;
                    container.set(0);
                    containerFollower.set(0);
                }
                else {
                    container.set(ArmConstants.containerMotorSpeedBottomDrop);
                    containerFollower.set(ArmConstants.containerMotorSpeedTopDrop);
                }
                break;
        }

        switch (state) { // Elevator
            case ResetHeight:
                RecallibrateHeight();
                break;
            case NormalOper:
                if(!elevator.getReverseLimitSwitch().isPressed() && getHeight()-ArmConstants.minHeight < ArmConstants.autoResetHeight) {
                    elevator.getExternalEncoder().setPosition(0);
                }
                double v = elevatorPID.calculate(getHeight(),targetHeight)+ArmConstants.elevatorMotorBias;
                v = Math.min(Math.max(v,-0.2),.3);
                elevator.set(v);
                break;
        }
    }
    @Override
    public void simulationPeriodic() {}
}
 