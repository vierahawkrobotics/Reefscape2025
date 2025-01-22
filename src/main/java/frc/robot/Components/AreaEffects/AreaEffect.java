package frc.robot.Components.AreaEffects;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Components.AreaEffects.AreaEffectShapes.DefaultShape;

public class AreaEffect {
    public DefaultShape shape;
    public Double maxArmHeight;
    public Double maxSpeed;
    public Pose2d targetPose;
    public Command onEnterCommand;
    public Command onExitCommand;
}