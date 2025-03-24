package frc.robot.Components.AreaEffects;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Components.AreaEffects.AreaEffectShapes.DefaultShape;

/**
 * A single Area Effect object
 * Area effects are objects that hold data that is conditionally outputted by
 * the Area Effect Handler determined by the shape passed
 * @author Darren Ringer
 */
public class AreaEffect {

    public AreaEffect() {}
    public AreaEffect(DefaultShape shape, Rotation2d rot) {
        this.shape = shape;
        targetPose = new Pose2d(0,0, rot);
    }
    public AreaEffect(DefaultShape shape, boolean autoAlign) {
        this.shape = shape;
        this.autoAlign = autoAlign;
    }

    public DefaultShape shape;
    public Double maxArmHeight;
    public Double maxSpeed;
    public Pose2d targetPose;
    public Boolean autoAlign;
    public Command onEnterCommand;
    public Command onExitCommand;
}