package frc.robot.Components;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Drivetrain.DrivetrainConstants;

public class ComponentManager {
    public static void initialize(){
        // NOTE: This needs to be changed eventually to reflect auto and such
        PositionComponent.initialize(DrivetrainConstants.kinematics, ()->{return Drivetrain.getInstance().getSwerveModulePositions();}, Pose2d.kZero);
    }
    public static void periodic(){
        PositionComponent.periodic();
    }
}
