package frc.robot.Components.AreaEffects;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Components.PositionComponent.PositionComponent;

/**
 * The area effect handler that holds all area effects and returns attributes based off of the current effect
 * It is a singleton class designed around utilizing static methods
 * @author Darren Ringer
 */
public class AreaEffectsHandler{
    private static AreaEffectsHandler instance;
    private static List<AreaEffect> areaEffects;
    private static AreaEffect currentEffect;

    private AreaEffectsHandler(){
        areaEffects = new ArrayList<AreaEffect>();
    }

    public static void initialize(){
        if (instance == null)
            instance = new AreaEffectsHandler();

        // input areas
        areaEffects.add(new AreaEffect(new AreaEffectShapes.Circle(-7.923022,-3.37058, .84),Rotation2d.fromDegrees(306)));
        areaEffects.add(new AreaEffect(new AreaEffectShapes.Circle(-7.923022,3.37058, .84),Rotation2d.fromDegrees(54)));
        // center area
        AreaEffect centerArea = new AreaEffect(new AreaEffectShapes.Circle(-4.284,0, 1));
        centerArea.autoAlign = true;
        areaEffects.add(centerArea);
    }

    public static AreaEffectsHandler getInstance(){
        if (instance == null)
            instance = new AreaEffectsHandler();
        return instance;
    }

    /**
     * Handles the periodic of the handler<p>
     * Updates current effect and calls enter/exit commands if applicable
     * @author Darren Ringer
     */
    public static void periodic(){
        Pose2d currentPose = PositionComponent.getRobotPose();
        double currentPointX = currentPose.getX();
        double currentPointY = currentPose.getY();
        if(currentEffect != null && !currentEffect.shape.check(currentPointX,currentPointY)){
            if(currentEffect.onExitCommand != null) {
                currentEffect.onExitCommand.schedule();
            }
            if(currentEffect.onEnterCommand != null) {
                currentEffect.onEnterCommand.cancel();
            }
            currentEffect = null;
        }
        if(currentEffect == null) {
            for(int i = 0; i < areaEffects.size(); i++) {
                if(areaEffects.get(i).shape.check(currentPointX,currentPointY)){
                    currentEffect = areaEffects.get(i);
                    if(currentEffect.onEnterCommand != null){
                        currentEffect.onEnterCommand.schedule();
                    }
                    break;
                }
            }
        }
    }

    /*
     Getter commands
     */
    public static Double getMaxArmHeight(){
        if(currentEffect == null) return null;
        return currentEffect.maxArmHeight;
    }
    public static Double getMaxSpeed(){
        if(currentEffect == null) return null;
        return currentEffect.maxSpeed;
    }
    public static Pose2d getTargetPose(){
        if(currentEffect == null) return null;
        return currentEffect.targetPose;
    }
    public static Command getEnterCommand(){
        if(currentEffect == null) return null;
        return currentEffect.onEnterCommand;
    }
    public static Command getExitCommand(){
        if(currentEffect == null) return null;
        return currentEffect.onExitCommand;
    }
    public static boolean isAreaEffect(){
        return !(currentEffect == null);
    }
    public static Boolean getIsAutoAlign() {
        if(currentEffect == null) return null;
        return currentEffect.autoAlign;
    }


}
