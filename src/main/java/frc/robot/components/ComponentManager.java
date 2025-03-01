package frc.robot.Components;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Components.AreaEffects.AreaEffectsHandler;
import frc.robot.Components.PositionComponent.PositionComponent;
import frc.robot.Components.PositionComponent.PositionComponentSettings;

public class ComponentManager {
    private static ShuffleboardTab posTab;
    public static void Initialize() {
        PositionComponent.initialize(new Pose2d());
        AreaEffectsHandler.initialize();
        HighwaySystem.initialize();

        //Shuffleboard stuff
        posTab = Shuffleboard.getTab("Position Data");
        posTab.addString("Estimated Velocity Vector", ()->{return String.format("(x: %.3f, y: %.3f, r: %.3f)",
            PositionComponent.getChassisSpeeds().vxMetersPerSecond,
            PositionComponent.getChassisSpeeds().vyMetersPerSecond,
            PositionComponent.getChassisSpeeds().omegaRadiansPerSecond
        );});
        posTab.addString("Estimated Gyro Velocity Vector", ()->{return String.format("(x: %.3f, y: %.3f, r: %.3f)",
            PositionComponent.getChassisSpeeds(PositionComponentSettings.velType.kGyroscope).vxMetersPerSecond,
            PositionComponent.getChassisSpeeds(PositionComponentSettings.velType.kGyroscope).vyMetersPerSecond,
            PositionComponent.getChassisSpeeds(PositionComponentSettings.velType.kGyroscope).omegaRadiansPerSecond
        );});
        posTab.addDouble("Drive Estimated X", ()->{return PositionComponent.getRobotPose().getX();});
        posTab.addDouble("Drive Estimated Y", ()->{return PositionComponent.getRobotPose().getY();});
        posTab.addDouble("Drive Estimated Rotation (degrees)", ()->{return PositionComponent.getRobotPose().getRotation().getDegrees();});
        posTab.addDouble("Gyroscope Estimated Rotation (degrees)", ()->{return PositionComponent.getGyroRotation();});

        posTab.addDouble("Limelight Estimated X", ()->{return LimelightComponent.getLastAprilTag().getX();});
        posTab.addDouble("Limelight Estimated Y", ()->{return LimelightComponent.getLastAprilTag().getY();});
        posTab.addDouble("Limelight Estimated Rotation (degrees)", ()->{return LimelightComponent.getLastAprilTag().getRotation().getDegrees();});

        posTab.addBoolean("Limelight Target Found", ()->{return LimelightComponent.active();});
    }
    public static void Periodic() {
        PositionComponent.periodic();
        AreaEffectsHandler.periodic();
    }
}
