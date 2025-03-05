package frc.robot.ArmSubsystem;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

enum ArmState {
    ResetHeight,
    NormalOper
}

enum AlgaeState {
    Rest,
    Eject
}

public class ArmSubsystem extends SubsystemBase {
    private ShuffleboardTab armTab;
    private ArmState state = ArmState.ResetHeight; // Immediately reset height
    private AlgaeState algaeState = AlgaeState.Rest;
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
    public double algaeStartTime;

    public ArmSubsystem() {
        armTab = Shuffleboard.getTab("Arm Subsystem");

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

        // Algae Motor Setup
        // algaeMotor = new SparkFlex(ArmConstants.algaeMotorID, MotorType.kBrushless);
        // SparkBaseConfig algaeConfig = new SparkFlexConfig();
        // algaeConfig.idleMode(IdleMode.kBrake);
        // algaeMotor.configure(algaeConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        // Shuffleboard Setup
        armTab.addNumber("Current Height", () -> {return curHeight;});
        armTab.addNumber("Target Height", () -> {return targetHeight;});
        armTab.addDouble("Elevator Radians", () -> {return elevatorFollower.getExternalEncoder().getPosition();});
        armTab.addString("Collector State", () -> {return intakeState.toString();});
        armTab.addBoolean("Limit Switch", () -> {return elevator.getReverseLimitSwitch().isPressed();});
        armTab.addNumber("Current Height Inches", () -> {return curHeight*39.37;});
    }
    
    /**
     * @param motorState new algae motorState (Active or Inactive)
     * @author Christian M
     */
    public void setAlgaeMotorSpeed(ArmConstants.AlgaeMotorState motorState){
        if (motorState == ArmConstants.AlgaeMotorState.Active & algaeState != AlgaeState.Eject) {
            algaeState = AlgaeState.Eject;
            algaeStartTime = Timer.getFPGATimestamp();
        } else {
            algaeState = AlgaeState.Rest;
        }
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
     * @return left/right offset in meters based on limit switch pressed (0 if none pressed)
     * @author Andrew S
     */
    public double isLimitSwitchPressed() {
        int pressed = 0;
        if (container.getForwardLimitSwitch().isPressed()) {
            pressed = 1;
        }
        if (container.getReverseLimitSwitch().isPressed()) {
            if(pressed != 0) System.out.println("ArmSubsystem.isLimitSwitchPressed - multiple limit switches pressed");
            pressed = 2;
        }
        if (containerFollower.getForwardLimitSwitch().isPressed()) {
            if(pressed != 0) System.out.println("ArmSubsystem.isLimitSwitchPressed - multiple limit switches pressed");
            pressed = 3;
        }
        if (containerFollower.getReverseLimitSwitch().isPressed()) {
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
        if (state == ArmConstants.IntakeState.Drop && intakeState != ArmConstants.IntakeState.Drop) {
            startTime = Timer.getFPGATimestamp();
        } else if (state == ArmConstants.IntakeState.Collect && intakeState != ArmConstants.IntakeState.Collect) {
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
            elevator.set(0);
        }
        else {
            elevator.set(ArmConstants.resetHeightModeBias);
        }
    }
    
    @Override
    public void periodic() {
        curHeight = elevatorFollower.getExternalEncoder().getPosition()*ArmConstants.gearRadius + ArmConstants.armHeight;
        
        switch (intakeState) { // Collect and Drop Periodic
            case Rest:
                container.set(0);
                containerFollower.set(0);
                break;
            case Collect:
                limitSwitchOffset = isLimitSwitchPressed();
                if (limitSwitchOffset != 0 || Timer.getFPGATimestamp()-startTime >= ArmConstants.containerCollectTime) {
                    intakeState = ArmConstants.IntakeState.Rest;
                    container.set(0);
                    containerFollower.set(0);
                }
                else {
                    container.set(ArmConstants.containerMotorSpeedBottomCollect);
                    containerFollower.set(ArmConstants.containerMotorSpeedTopCollect);
                }
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

        switch (state) { // Elevator height periodic
            case ResetHeight:
                RecallibrateHeight();
                break;
            case NormalOper:
                if(!elevator.getReverseLimitSwitch().isPressed() && getHeight()-ArmConstants.armHeight < ArmConstants.autoResetHeight) {
                    elevator.getExternalEncoder().setPosition(0);
                }
                double v = elevatorPID.calculate(getHeight(),targetHeight)+ArmConstants.elevatorMotorBias;
                v = Math.min(Math.max(v,-0.2),.3);
                elevator.set(v);
                break;
        }

        switch(algaeState) {
            case Rest:
                algaeMotor.set(0);
                break; 

            case Eject:
                if (Timer.getFPGATimestamp()-algaeStartTime >= ArmConstants.algaeEjectTime) {
                    algaeState = AlgaeState.Rest;
                    algaeMotor.set(0);
                }
                else {
                    algaeMotor.set(ArmConstants.algaeMotorSpeed);
                }
                break;
        }
    }
    @Override
    public void simulationPeriodic() {}
}
 