package frc.robot.Components.AreaEffects;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import frc.robot.Components.AreaEffects.AreaEffectShapes.*;

public class AreaEffects {
    public static AreaEffect test1= new AreaEffect(
        new Polygon(List.of(
            new Point(0,0)
        )),
        Map.of(
            "Test", Double.valueOf(1.2)
        )
    );
}
