package org.frogforce503.lib.util;

import org.frogforce503.robot2024.subsystems.drive.Drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class OrientedTranslation {
    Translation2d translation;
    boolean fieldCentric;

    public OrientedTranslation(Translation2d translation, boolean fieldCentric) {
        this.translation = translation;
        this.fieldCentric = fieldCentric;
    }

    public OrientedTranslation(Translation2d translation2d) {
        this(translation2d, false);
    }

    public OrientedTranslation() {
        this(new Translation2d());
    }
    
    public Translation2d get() {
        if (fieldCentric) {
            ChassisSpeeds converted = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), 0, Drive.getInstance().getAngle());
            return new Translation2d(converted.vxMetersPerSecond, converted.vyMetersPerSecond);
        }
        
        return translation;
    }
}