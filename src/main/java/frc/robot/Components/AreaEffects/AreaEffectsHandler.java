package frc.robot.Components.AreaEffects;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Components.PositionComponent;
import frc.robot.Components.AreaEffects.AreaEffectShapes.Point;

public class AreaEffectsHandler{
    private static AreaEffectsHandler instance;
    private static List<AreaEffect> areaEffects;
    private static AreaEffect currentEffect;

    private AreaEffectsHandler(){
        areaEffects = new ArrayList<AreaEffect>();
    }

    public static AreaEffectsHandler getInstance(){
        if (instance == null)
            instance = new AreaEffectsHandler();
        return instance;
    }

    public static Pose2d getTargetPose(){
        return currentEffect.targetPose;
    }

    public static void periodic(){
        Pose2d currentPose = PositionComponent.getRobotPose();
        Point currentPoint = new Point(currentPose.getX(),currentPose.getY());
        boolean exit = false;
        if(currentEffect == null || !currentEffect.shape.check(currentPoint)){
            for(int i=0; i<areaEffects.size();i++){
                if(areaEffects.get(i).shape.check(currentPoint)){
                    currentEffect = areaEffects.get(i);
                    exit = true;
                }
            }
            if(!exit){
                currentEffect = null;
            }
        }
    }
}
