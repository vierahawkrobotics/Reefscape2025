package frc.robot.Components.AreaEffects;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Components.PositionComponent;

/**
 * The area effect handler that holds all area effects and returns attributes based off of the current effect
 * @autor Darren Ringer
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
        boolean exit = false;
        if(currentEffect.onExitCommand != null && !currentEffect.shape.check(currentPointX,currentPointY)){
            currentEffect.onExitCommand.schedule();
        }
        if(currentEffect == null || !currentEffect.shape.check(currentPointX,currentPointY)){
            for(int i=0; i<areaEffects.size();i++){
                if(areaEffects.get(i).shape.check(currentPointX,currentPointY)){
                    currentEffect = areaEffects.get(i);
                    exit = true;
                    if(currentEffect.onEnterCommand != null){
                        currentEffect.onEnterCommand.schedule();
                    }
                    break;
                }
            }
            if(!exit){
                currentEffect = null;
            }
        }
    }

    /*
     Getter commands
     */
    public static Double getMaxArmHeight(){
        return currentEffect.maxArmHeight;
    }
    public static Double getMaxSpeed(){
        return currentEffect.maxSpeed;
    }
    public static Pose2d getTargetPose(){
        return currentEffect.targetPose;
    }
    public static Command getEnterCommand(){
        return currentEffect.onEnterCommand;
    }
    public static Command getExitCommand(){
        return currentEffect.onExitCommand;
    }



}
