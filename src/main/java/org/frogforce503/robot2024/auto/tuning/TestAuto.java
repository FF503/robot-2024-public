package org.frogforce503.robot2024.auto.tuning;

import org.frogforce503.lib.auto.AutoMode;
import org.frogforce503.lib.trajectory.PlannedPath;
import org.frogforce503.lib.trajectory.Route;
import org.frogforce503.lib.trajectory.SwervePathBuilder;
import org.frogforce503.lib.trajectory.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class TestAuto extends AutoMode {

    PlannedPath path;

    public TestAuto() {
        path = SwervePathBuilder.generate(4.0, 3.0,
            Waypoint.fromHolonomicPose(new Pose2d(1, 1, new Rotation2d())),
            Waypoint.fromHolonomicPose(new Pose2d(2, 1.5, new Rotation2d())),
            Waypoint.fromHolonomicPose(new Pose2d(3, 1.5, new Rotation2d()))
        );
    }

    @Override
    public Command routine() {
        return drive(path);
    }

    @Override
    public Route getRoute() {
        return new Route(path);
    }
}