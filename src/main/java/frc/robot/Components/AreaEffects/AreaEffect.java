package frc.robot.Components.AreaEffects;

import java.util.HashMap;

import frc.robot.Components.AreaEffects.AreaEffectShapes.DefaultShape;

/**
 * A single Area Effect object
 * Area effects are objects that hold data that is conditionally outputted by
 * the Area Effect Handler determined by the shape passed
 * @author Darren Ringer
 */
public class AreaEffect {
    public AreaEffect(DefaultShape shape, HashMap<String, ? extends Object> data){
        this.shape = shape;
        this.data = data;
    }
    public DefaultShape shape;
    public HashMap<String, ? extends Object> data;
}