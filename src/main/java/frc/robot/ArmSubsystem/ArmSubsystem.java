package frc.robot.ArmSubsystem;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ArmSubsystem.ArmConstants.HeightState;

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
    public SparkFlex algaeMotor; 
    private double targetHeight = ArmConstants.armHeight;
    private double curHeight = 0;
    PIDController elevatorPID = new PIDController(ArmConstants.elevatorP, ArmConstants.elevatorI, ArmConstants.elevatorD);
    public double limitSwitchOffset;
    public double startTime;

    private double elevatorSet = 0;
    private double collectorSet = 0;

    public ArmSubsystem() {
        armTab = Shuffleboard.getTab("Arm Subsystem");

        // Create and setup motors for Elevator
        elevator = new SparkFlex(ArmConstants.elevatorMotorID, MotorType.kBrushless);
        elevatorFollower = new SparkFlex(ArmConstants.elevatorFollowMotorID, MotorType.kBrushless);
        SparkFlexConfig elevatorConfig = new SparkFlexConfig();
        elevatorConfig.idleMode(IdleMode.kBrake);
        elevatorConfig.limitSwitch
            .reverseLimitSwitchEnabled(false);
        elevator.configure(elevatorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        SparkFlexConfig elevatorFollowerConfig = new SparkFlexConfig();
        elevatorFollowerConfig.externalEncoder
            .measurementPeriod(20)
            .positionConversionFactor(ArmConstants.encoderPositionFactor)
            .velocityConversionFactor(ArmConstants.encoderVelocityFactor);
        elevatorFollowerConfig
            .follow(elevator, true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ArmConstants.maxAmp);
        elevatorFollower.configure(elevatorFollowerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        // Create and setup motors for Drop and Collect
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

        // Create and setup motor for Algae
        // algaeMotor = new SparkFlex(ArmConstants.algaeMotorID, MotorType.kBrushless);
        // SparkBaseConfig algaeConfig = new SparkFlexConfig();
        // algaeConfig.idleMode(IdleMode.kBrake);
        // algaeMotor.configure(algaeConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        
        armTab.addNumber("current height", () -> {return curHeight;});
        armTab.addNumber("target height", () -> {return targetHeight;});
        armTab.addDouble("elevator motor set", () -> {return elevatorSet;});
        armTab.addDouble("collector motor set", () -> {return collectorSet;});
        armTab.addDouble("elevator rad", () -> {return elevatorFollower.getExternalEncoder().getPosition();});
        armTab.addString("elevator state", () -> {return state.toString();});
        armTab.addString("collector state", () -> {return intakeState.toString();});
        armTab.addBoolean("limit switch", () -> {return elevator.getReverseLimitSwitch().isPressed();});
    }
    
    /**
     * THIS FUNCTION IS NOT CORRECT PLEASE FIX
     * Set algae motor speed
     * @param motorState new algae motorState (Active or Inactive)
     * @author Christian M
     */
    public void setAlgaeMotorSpeed(ArmConstants.AlgaeMotorState motorState){
       // algaeMotor.set(motorState.getMotorState());
    }

    /**
     * Move elevator arm up to eject algae
     * @param height target HeightState
     * @author Andrew S
     */
    public void setHeightState(ArmConstants.HeightState height) {
        SetTargetHeight(height.getHeight());
    }

    /**
     * @return left/right offset in meters based on limit switch pressed (0 if none pressed)
     * @author Andrew S
     */
    public double isLimitSwitchPressed() {
        int pressed = 0;
        if(container.getForwardLimitSwitch().isPressed()) {
            pressed = 1;
        }
        if(container.getReverseLimitSwitch().isPressed()) {
            if(pressed != 0) System.out.println("ArmSubsystem.isLimitSwitchPressed - multiple limit switches pressed");
            pressed = 2;
        }
        if(containerFollower.getForwardLimitSwitch().isPressed()) {
            if(pressed != 0) System.out.println("ArmSubsystem.isLimitSwitchPressed - multiple limit switches pressed");
            pressed = 3;
        }
        if(containerFollower.getReverseLimitSwitch().isPressed()) {
            if(pressed != 0) System.out.println("ArmSubsystem.isLimitSwitchPressed - multiple limit switches pressed");
            pressed = 4;
        }
        if (pressed == 1) { // Far left channel pressed
            return ArmConstants.farLeftIntakeChannel;
        } else if (pressed == 2) { // Middle left channel pressed
            return ArmConstants.middleLeftIntakeChannel;
        } else if (pressed == 3) { // Middle right channel pressed
            return ArmConstants.middleRightIntakeChannel;
        } else if (pressed == 4) { // Far right channel pressed
            return ArmConstants.farRightIntakeChannel;
        } else { // none pressed
            return 0;
        }
    }

    /**
     * Change current IntakeState and start timer if changing to Drop state
     * @param state new intake state
     * @author Andrew S
     */
    public void setIntakeState(ArmConstants.IntakeState state) {
        if(state == ArmConstants.IntakeState.Drop && intakeState != ArmConstants.IntakeState.Drop) {
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
        this.targetHeight = Math.min(Math.max(targetHeight,ArmConstants.armHeight),ArmConstants.maxHeight);
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
            elevatorSet = 0;
            elevator.set(0);
        }
        else {
            elevatorSet = ArmConstants.resetHeightModeBias;
            elevator.set(ArmConstants.resetHeightModeBias);
        }
    }
    
    @Override
    public void periodic() {
        curHeight = elevatorFollower.getExternalEncoder().getPosition()*ArmConstants.gearRadius + ArmConstants.armHeight;
        
        switch (intakeState) { // Periodic for Collect and Drop commands
            case Rest:
                container.set(0);
                containerFollower.set(0);
                collectorSet = 0;
                break;
            case Collect:
                limitSwitchOffset = isLimitSwitchPressed();
                if (limitSwitchOffset != 0) {
                    intakeState = ArmConstants.IntakeState.Rest;
                    collectorSet = 0;
                    container.set(0);
                    containerFollower.set(0);
                }
                else {
                    container.set(ArmConstants.containerMotorSpeedBottomCollect);
                    containerFollower.set(ArmConstants.containerMotorSpeedTopCollect);
                    collectorSet = -ArmConstants.containerMotorSpeedBottomCollect;
                }
                break;
            case Drop:
                if (Timer.getFPGATimestamp()-startTime >= ArmConstants.containerDropTime) {
                    intakeState = ArmConstants.IntakeState.Rest;
                    container.set(0);
                    containerFollower.set(0);
                    collectorSet = 0;
                }
                else {
                    container.set(ArmConstants.containerMotorSpeedBottomDrop);
                    containerFollower.set(ArmConstants.containerMotorSpeedTopDrop);
                    collectorSet = ArmConstants.containerMotorSpeedBottomDrop;
                }
                break;
        }

        switch (state) { // Elevator height periodic
            case ResetHeight:
                RecallibrateHeight();
                break;
            case NormalOper:
                if(!elevator.getReverseLimitSwitch().isPressed()) {
                    elevator.getExternalEncoder().setPosition(0);
                }
                double v = elevatorPID.calculate(getHeight(),targetHeight)+ArmConstants.elevatorMotorBias;
                elevatorSet = v;
                v = Math.min(Math.max(v,-0.15),.15);
                elevator.set(v);
                break;
        }
    }
    @Override
    public void simulationPeriodic() {}
}
 