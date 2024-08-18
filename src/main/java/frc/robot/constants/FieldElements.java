package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.util.Util;

public class FieldElements { 
    /*INCHES
     *https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024FieldDrawings.pdf p.240
     *AND
     *https://firstfrc.blob.core.windows.net/frc2024/Manual/Sections/2024GameManual-05ARENA.pdf
     */
    public static final Translation3d kSpeakerHole = Util.isBlue() ? 
                                                new Translation3d(-1.5, 218.42, 80) :
                                                new Translation3d(652.73, 218.42, 80);
    // consider changing to meters based on what botpose reads as?
}
