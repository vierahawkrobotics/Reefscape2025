package frc.robot.ArmSubsystem;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
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

public class ArmSubsystem extends SubsystemBase {
    private ShuffleboardTab armTab;
    private ArmState state = ArmState.ResetHeight; // Immediately reset height
    private ArmConstants.IntakeState intakeState = ArmConstants.IntakeState.Rest;
    private SparkFlex elevator;
    private SparkFlex elevatorFollower;
    public SparkFlex container;
    public SparkFlex containerFollower;
    public SparkFlex algaeMotor; 
    private double targetHeight = ArmConstants.ground;
    private double curHeight = 0;
    PIDController elevatorPID = new PIDController(ArmConstants.elevatorP, ArmConstants.elevatorI, ArmConstants.elevatorD);
    private double limitSwitchOffset;
    public double startTime;

    public ArmSubsystem() {
        armTab = Shuffleboard.getTab("Arm Subsystem");

        // Create and setup motors for Elevator
        elevator = new SparkFlex(ArmConstants.elevatorMotorID, MotorType.kBrushless);
        elevatorFollower = new SparkFlex(ArmConstants.elevatorFollowMotorID, MotorType.kBrushless);
        SparkBaseConfig elevatorConfig = new SparkFlexConfig();
        elevatorConfig.idleMode(IdleMode.kBrake);
        elevatorConfig.encoder
            .positionConversionFactor(ArmConstants.encoderPositionFactor)
            .velocityConversionFactor(ArmConstants.encoderVelocityFactor);
        elevator.configure(elevatorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        SparkBaseConfig elevatorFollowerConfig = new SparkFlexConfig();
        elevatorFollowerConfig
            .follow(elevator)
            .idleMode(IdleMode.kBrake);
        elevatorFollower.configure(elevatorFollowerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        // Create and setup motors for Drop and Collect
        container = new SparkFlex(ArmConstants.containerMotorID, MotorType.kBrushless);
        containerFollower = new SparkFlex(ArmConstants.containerFollowMotorID, MotorType.kBrushless);
        SparkBaseConfig containerConfig = new SparkFlexConfig();
        containerConfig.idleMode(IdleMode.kBrake);
        container.configure(containerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        SparkBaseConfig containerFollowerConfig = new SparkFlexConfig();
        containerFollowerConfig
            .follow(container)
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        containerFollower.configure(containerFollowerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        // Create and setup motor for Algae
        algaeMotor = new SparkFlex(ArmConstants.algaeMotorID, MotorType.kBrushless);
        SparkBaseConfig algaeConfig = new SparkFlexConfig();
        algaeConfig.idleMode(IdleMode.kBrake);
        algaeMotor.configure(algaeConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

    }
    
    /**
     * Set algae motor speed
     * @param motorState new algae motorState (Active or Inactive)
     * @author Christian M
     */
    public void setAlgaeMotorSpeed(ArmConstants.AlgaeMotorState motorState){
        algaeMotor.set(motorState.getMotorState());
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
        System.out.println(pressed);
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
        targetHeight = Math.min(Math.max(targetHeight,ArmConstants.armHeight),ArmConstants.maxHeight);
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
        return Math.abs(targetHeight - elevator.getExternalEncoder().getPosition()) < ArmConstants.epsilon;
    }

    /**
     * Reset height to 0
     * @author Andrew S
     */
    private void RecallibrateHeight() {
        elevator.set(ArmConstants.resetHeightModeBias);
        if(elevator.getForwardLimitSwitch().isPressed()) {
            elevator.getExternalEncoder().setPosition(0);
            state = ArmState.NormalOper;
        }
    }
    
    @Override
    public void periodic() {
        curHeight = elevator.getExternalEncoder().getPosition()*2*Math.PI*ArmConstants.gearRadius + ArmConstants.armHeight;
        switch (intakeState) { // Periodic for Collect and Drop commands
            case Rest:
                container.set(0);
                break;
            case Collect:
                limitSwitchOffset = isLimitSwitchPressed();
                if (limitSwitchOffset != 0) {
                    intakeState = ArmConstants.IntakeState.Rest;
                    container.set(0);
                }
                else {
                    container.set(ArmConstants.containerMotorSpeed);
                }
                break;
            case Drop:
                if (Timer.getFPGATimestamp()-startTime >= ArmConstants.containerDropTime) {
                    intakeState = ArmConstants.IntakeState.Rest;
                    container.set(0);
                }
                else {
                    container.set(-ArmConstants.containerMotorSpeed);
                }
                break;
        }

        switch (state) { // Elevator height periodic
            case ResetHeight:
                RecallibrateHeight();
                break;
            case NormalOper:
                elevator.set(elevatorPID.calculate(getHeight(),targetHeight)+ArmConstants.elevatorMotorBias);
                break;
        }
    }
    @Override
    public void simulationPeriodic() {}
}
 