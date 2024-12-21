package org.frogforce503.lib.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveDriveState {
    public int SuccessfulDaqs;
    public int FailedDaqs;
    public Pose2d Pose;
    public SwerveModuleState[] ModuleStates;
    public double OdometryPeriod;

    public static SwerveDriveState translate(com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState other) {
        SwerveDriveState state = new SwerveDriveState();
        state.SuccessfulDaqs = other.SuccessfulDaqs;
        state.FailedDaqs = other.FailedDaqs;
        state.Pose = other.Pose;
        state.ModuleStates = other.ModuleStates;
        state.OdometryPeriod = other.OdometryPeriod;
        
        return state;
    }
}