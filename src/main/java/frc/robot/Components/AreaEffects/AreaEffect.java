package frc.robot.Components.AreaEffects;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Components.AreaEffects.AreaEffectShapes.DefaultShape;

public class AreaEffect {
    //Have attribute getting/setting capabilities
    //Handler returns get/set of currently cached value
    public DefaultShape shape;
    public float maxArmHeight;
    public float maxSpeed;
    public Pose2d targetPose;
}