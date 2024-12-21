package org.frogforce503.robot2024.auto.comp.test;

import org.frogforce503.lib.auto.AutoMode;
import org.frogforce503.lib.trajectory.PlannedPath;
import org.frogforce503.lib.trajectory.Route;
import org.frogforce503.lib.trajectory.SwervePathBuilder;
import org.frogforce503.lib.trajectory.Waypoint;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class AutonTest extends AutoMode {

    private PlannedPath go;

    public AutonTest() {
        go = SwervePathBuilder.generate(2, 1,
            new Waypoint(),
            new Waypoint(new Translation2d(Units.feetToMeters(3), 0))
        );
    }

    @Override
    public Command routine() {
        return drive(go);
    }

    @Override
    public Route getRoute() {
        return new Route(go);
    }
    
}