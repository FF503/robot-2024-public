package org.frogforce503.lib.drawing;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public class NumberTrajectories {
    public static double SCALE = 1.0; // meters

    public static final Trajectory NUMBER_1, NUMBER_2, NUMBER_3;
    static {
        TrajectoryConfig base = new TrajectoryConfig(SCALE, SCALE);
        
        NUMBER_1 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, -SCALE/2, new Rotation2d(Math.PI/2)), List.of(new Translation2d(0, 0)), new Pose2d(0, SCALE/2, new Rotation2d(Math.PI/2)), base
        );

        SCALE *= 0.5;

        NUMBER_2 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(SCALE, -SCALE/2, new Rotation2d(Math.PI)), List.of(
                new Translation2d(0, -SCALE/2),
                new Translation2d(SCALE/2, 0),
                new Translation2d(SCALE, SCALE/2)
            ), new Pose2d(0, SCALE/2, new Rotation2d(Math.PI)), base
        );

        NUMBER_3 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, -SCALE/2, new Rotation2d(0)), List.of(
                new Translation2d(SCALE, -SCALE/4),
                new Translation2d(0, 0),
                new Translation2d(SCALE, SCALE/4)
            ), new Pose2d(0, SCALE/2, new Rotation2d(0)), base
        );
    }
    
}
