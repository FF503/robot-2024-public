package org.frogforce503.robot2024.auto.tuning;

import org.frogforce503.lib.auto.AutoMode;
import org.frogforce503.lib.trajectory.PlannedPath;
import org.frogforce503.lib.trajectory.Route;
import org.frogforce503.lib.trajectory.SwervePathBuilder;
import org.frogforce503.lib.trajectory.Waypoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class TuningAuton extends AutoMode{

    private PlannedPath s1;
    private double length = Units.feetToMeters(6);

    public TuningAuton() {
        s1 = SwervePathBuilder.generate(5, 3, 
            new Waypoint(),
            new Waypoint(new Translation2d(length, 0)),
            new Waypoint(new Translation2d(length, length)),
            new Waypoint(new Translation2d(0, length)),
            new Waypoint()
        );
    }

    @Override
    public Command routine() {
        return drive(s1);
    }

    @Override
    public Route getRoute() {
        return new Route(s1);
    }
    
}