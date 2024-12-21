package org.frogforce503.lib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleStateEx extends SwerveModuleState {
    private ThrottleUnit throttleUnit;
    
    public SwerveModuleStateEx(double speedMetersPerSecond, Rotation2d angle, ThrottleUnit throttleUnit) {
        super(speedMetersPerSecond, angle);
        this.throttleUnit = throttleUnit;
    }

    public SwerveModuleStateEx(double speedMetersPerSecond, Rotation2d angle) {
        this(speedMetersPerSecond, angle, ThrottleUnit.METERS_PER_SECOND);
    }

    public SwerveModuleStateEx() {
        this(0, new Rotation2d(), ThrottleUnit.METERS_PER_SECOND);
    }

    public SwerveModuleStateEx(SwerveModuleState state) {
        this(state.speedMetersPerSecond, state.angle);
    }

    public ThrottleUnit getThrottleUnit() {
        return throttleUnit;
    }

    public static SwerveModuleStateEx[] convertList(SwerveModuleState[] states) {
        SwerveModuleStateEx[] exs = new SwerveModuleStateEx[states.length];
        for (int i = 0; i < exs.length; i++) {
            exs[i] = new SwerveModuleStateEx(states[i]);
        }
        return exs;
    }

    public static enum ThrottleUnit {
        METERS_PER_SECOND,
        VOLTS,
        PERCENT_OUT
    }
}
