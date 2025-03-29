package frc.robot.Components;

import java.util.Optional;
import java.util.OptionalInt;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Components.AreaEffects.AreaEffectsHandler;
import frc.robot.Components.CANdleComponent.CANdleController;
import frc.robot.Components.PositionComponent.PositionComponent;
import frc.robot.Components.PositionComponent.PositionComponentSettings;
import frc.robot.Components.PositionTools.PositionTools;

public class ComponentManager {
    private static double[] defaultArray = new double[11];
    private static ShuffleboardTab posTab;
    public static void Initialize() {
        CANdleController.initialize();
        PositionComponent.initialize();
        AreaEffectsHandler.initialize();
        HighwaySystem.initialize();
        PDHManager.Initialize();

        //Shuffleboard stuff
        posTab = Shuffleboard.getTab("Position Data");
        /*posTab.addString("Estimated Velocity Vector", ()->{return String.format("(x: %.3f, y: %.3f, r: %.3f)",
            PositionComponent.getChassisSpeeds().vxMetersPerSecond,
            PositionComponent.getChassisSpeeds().vyMetersPerSecond,
            PositionComponent.getChassisSpeeds().omegaRadiansPerSecond
        );});
        posTab.addString("Estimated Gyro Velocity Vector", ()->{return String.format("(x: %.3f, y: %.3f, r: %.3f)",
            PositionComponent.getChassisSpeeds(PositionComponentSettings.velType.kGyroscope).vxMetersPerSecond,
            PositionComponent.getChassisSpeeds(PositionComponentSettings.velType.kGyroscope).vyMetersPerSecond,
            PositionComponent.getChassisSpeeds(PositionComponentSettings.velType.kGyroscope).omegaRadiansPerSecond
        );});*/
        // posTab.addDouble("Pose Estimated X", ()->{return PositionComponent.getRobotPose().getX();});
        // posTab.addDouble("Pose Estimated Y", ()->{return PositionComponent.getRobotPose().getY();});
        // posTab.addDouble("Pose Estimated Rotation (degrees)", ()->{return PositionComponent.getRobotPose().getRotation().getDegrees();});
        //posTab.addDouble("Gyro R", ()->{return PositionComponent.getOffsetGyroRotation();});
        //posTab.addDouble("gyro offset", ()->{return PositionComponent.gyroOffset * 180 / Math.PI;});

        //posTab.addDouble("Limelight Estimated X", ()->{return LimelightComponent.getLastAprilTag().getX();});
        //posTab.addDouble("Limelight Estimated Y", ()->{return LimelightComponent.getLastAprilTag().getY();});
       // posTab.addDouble("Limelight Estimated Rotation (degrees)", ()->{return LimelightComponent.getLastAprilTag().getRotation().getDegrees();});

        //posTab.addBoolean("Limelight Target Found", ()->{return LimelightComponent.active();});
        //posTab.addBoolean("AprilTag Validity", ()->{return LimelightComponent.tagIsValid();});
        //posTab.addDouble("Limelight Distance", ()->{return LimelightComponent.dist;});
        posTab.addBoolean("In area effect", ()->{return AreaEffectsHandler.isAreaEffect();});
    }
    public static void Periodic() {
        CANdleController.periodic();
        PositionComponent.periodic();
        AreaEffectsHandler.periodic();
        PDHManager.Periodic();
    }
}
