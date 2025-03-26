package frc.robot.ArmSubsystem;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ArmSubsystem.ArmConstants.HeightState;
import frc.robot.Components.PositionComponent.PositionComponent;
import frc.robot.Match.RobotState;

enum ArmState {
    ResetHeight,
    IntakeHeight,
    NormalOper
}

public class ArmSubsystem extends SubsystemBase {
    private ShuffleboardTab armTab;
    private ArmState state = ArmState.ResetHeight; // Immediately reset height
    private ArmConstants.AlgaeMotorState algaeState = ArmConstants.AlgaeMotorState.Inactive;
    private ArmConstants.IntakeState intakeState = ArmConstants.IntakeState.Rest;
    private SparkFlex elevator;
    private SparkFlex elevatorFollower;
    public SparkFlex container;
    public SparkFlex containerFollower;
    public SparkFlex algaeMotor; 
    private double targetHeight = ArmConstants.minHeight;
    private double curHeight = 0;
    PIDController elevatorPID = new PIDController(ArmConstants.elevatorP, ArmConstants.elevatorI, ArmConstants.elevatorD);
    public Double limitSwitchOffset;
    public double startTime;
    public double algaeStartTime;

    public ArmSubsystem() {
        // Shuffleboard Setup
        armTab = Shuffleboard.getTab("Arm Subsystem");
        /*
        armTab.addNumber("Current Height", () -> {return curHeight;});
        armTab.addNumber("Current Height Inches", () -> {return curHeight*39.37;});
        armTab.addDouble("Elevator Height Radians", () -> {return elevatorFollower.getExternalEncoder().getPosition();});
        armTab.addNumber("Target Height", () -> {return targetHeight;});
        armTab.addString("Collector State", () -> {return intakeState.toString();});
        armTab.addString("Elv State", () -> {return state.toString();});
        armTab.addString("Algae State", () -> {return algaeState.toString();});
        armTab.addBoolean("Limit Switch", () -> {return elevator.getReverseLimitSwitch().isPressed();});*/
        armTab.addNumber("Elevator Limit Offset", () -> {
            if(getLimitSwitchOffset() == null) return 0;
            return getLimitSwitchOffset().doubleValue();
        });
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
            .measurementPeriod(40)
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
        algaeMotor = new SparkFlex(ArmConstants.algaeMotorID, MotorType.kBrushless);
        SparkFlexConfig algaeConfig = new SparkFlexConfig();
        algaeConfig.idleMode(IdleMode.kBrake);
        algaeMotor.configure(algaeConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    }
    
    /**
     * Change algae motor rotation speed based on motorState
     * @param motorState new algae motorState (Active or Inactive)
     * @author Christian M
     * @author Andrew S
     */
    public void setAlgaeMotorSpeed(ArmConstants.AlgaeMotorState motorState){
        if (motorState == ArmConstants.AlgaeMotorState.ActiveTemp && algaeState != motorState) {
            algaeStartTime = Timer.getFPGATimestamp();
        }
        algaeState = motorState;
    }

    /**
     * @return current algae state
     * @author Andrew S
     */
    public ArmConstants.AlgaeMotorState getAlgaeState() {
        return algaeState;
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
     * Move elevator arm to height registered with HeightState
     * @param height target HeightState
     * @author Andrew S
     */
    public void setHeightState(ArmConstants.HeightState height, boolean useAutoHeight) {
        SetTargetHeight(height.getHeight());
        if(!useAutoHeight) return;
        if(state != ArmState.ResetHeight && height == HeightState.Collect) state = ArmState.IntakeHeight;
    }

    /**
     * @return left/right offset in meters based on limit switch pressed (0 if none pressed)
     * @author Andrew S
     */
    public Double getLimitSwitchOffset() {
        return limitSwitchOffset;
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
     * @return current intake  
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
        if(state != ArmState.ResetHeight) state = ArmState.NormalOper;
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
        
        Pose2d robot = PositionComponent.getRobotPose();
        
        if(state == ArmState.IntakeHeight) {
            double x = robot.getX() + 7.923;
            double y = robot.getY() + 3.371;
            double d1 = Math.cos(-0.942) * x + Math.sin(-0.942) * y;
            x = robot.getY() - 3.371;
            double d2 = Math.cos(-5.341) * x + Math.sin(-5.341) * y;
            SetTargetHeight(0.951-Math.min(d1,d2) * Math.tan(0.611));
        }

        //limit switch
        double sum = 0;
        int total = 0;
        if (!container.getForwardLimitSwitch().isPressed()) {
            total++;
            sum += ArmConstants.farLeftIntakeChannel;
        }
        if (!container.getReverseLimitSwitch().isPressed()) {
            total++;
            sum += ArmConstants.middleLeftIntakeChannel;
        }
        if (!containerFollower.getForwardLimitSwitch().isPressed()) {
            total++;
            sum += ArmConstants.middleRightIntakeChannel;
        }
        if (!containerFollower.getReverseLimitSwitch().isPressed()) {
            total++;
            sum +=ArmConstants.farRightIntakeChannel;
        }
        if(total == 0) limitSwitchOffset = null;
        else limitSwitchOffset = sum / total;


        switch (intakeState) { // Collect and Drop
            case Rest:
                container.set(0);
                containerFollower.set(0);
                break;
            case Collect:
                if (limitSwitchOffset != null) {
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

        double v;
        switch (state) { // Elevator
            case ResetHeight:
                RecallibrateHeight();
                break;
            case IntakeHeight:
            case NormalOper:
                if(!elevator.getReverseLimitSwitch().isPressed() && getHeight()-ArmConstants.minHeight < ArmConstants.autoResetHeight) {
                    elevator.getExternalEncoder().setPosition(0);
                }
                v = elevatorPID.calculate(getHeight(),targetHeight + -RobotState.controller2.getLeftY() * 0.0254)+ArmConstants.elevatorMotorBias;
                v = Math.min(Math.max(v,-0.2),.3);
                elevator.set(v);
                break;
        }

        switch(algaeState) { // Algae
            case Inactive:
                algaeMotor.set(0);
                break; 
            case Active:
            case ActiveTemp:
                if (Timer.getFPGATimestamp()-algaeStartTime >= ArmConstants.algaeEjectTime) {
                    algaeState = ArmConstants.AlgaeMotorState.Inactive;
                    algaeMotor.set(0);
                } else {
                    algaeMotor.set(ArmConstants.algaeMotorSpeed);
                }
                break;
        }
    }
    @Override
    public void simulationPeriodic() {}
}
 