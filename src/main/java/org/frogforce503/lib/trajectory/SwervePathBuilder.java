package org.frogforce503.lib.trajectory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.frogforce503.lib.util.AllianceFlipUtil;
import org.frogforce503.robot2024.Robot;
import org.frogforce503.robot2024.fields.FieldConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;

// static wrapper for the CustomTrajectoryGenerator for easier use
public class SwervePathBuilder {

    public static PlannedPath generate(TrajectoryConfig config, List<Waypoint> waypoints) {
        CustomTrajectoryGenerator generator = new CustomTrajectoryGenerator();

        List<Waypoint> flippedWaypoints = new ArrayList<>();
        for (Waypoint waypoint : waypoints) {
            flippedWaypoints.add(AllianceFlipUtil.apply(waypoint));
        }

        try {
            generator.generate(config, waypoints);
        } catch (TrajectoryGenerationException exception) {
            System.out.print("TRAJECTORY GENERATION FAILED");
            exception.printStackTrace();
            return null;
        }
        
        return generator.getPlannedPath(waypoints);
    }

    public static PlannedPath generate(TrajectoryConfig config, Waypoint... waypoints) {
        return generate(config, Arrays.asList(waypoints));
    }

    public static PlannedPath generate(double vMax, double aMax, double vInitial, double vFinal, List<Waypoint> waypoints) {
        return generate(makeConfig(vMax, aMax, vInitial, vFinal), waypoints);
    }

    public static PlannedPath generate(double vMax, double aMax, List<Waypoint> waypoints) {
        return generate(vMax, aMax, 0.0, 0.0, waypoints);
    }

    public static PlannedPath generate(double vMax, double aMax, double vInitial, double vFinal, Waypoint... waypoints) {
        return generate(vMax, aMax, vInitial, vFinal, Arrays.asList(waypoints));
    }

    public static PlannedPath generate(double vMax, double aMax, Waypoint... waypoints) {
        return generate(vMax, aMax, 0.0, 0.0, Arrays.asList(waypoints));
    }

    public static PlannedPath regenerate(double vMax, double aMax, double vi, double vf, PlannedPath existing) {
        return generate(vMax, aMax, vi, vf, existing.getWaypoints());
    }

    public static PlannedPath reversedOf(PlannedPath other, double vMax, double aMax, double vi, double vf) {
        List<Waypoint> waypoints = other.getWaypoints();
        List<Waypoint> reversedWaypoints = new ArrayList<>();
        for (int i = waypoints.size() - 1; i >= 0; i--) {
            Waypoint w = waypoints.get(i);
            if (w.getDriveRotation().isPresent()) {
                w = new Waypoint(w.getTranslation(), w.getDriveRotation().get().plus(new Rotation2d(Math.PI)), w.getHolonomicRotation().isEmpty() ? null : w.getHolonomicRotation().get());
            }
            reversedWaypoints.add(w);
        }
        return generate(vMax, aMax, vi, vf, reversedWaypoints);
    }

    public static PlannedPath reversedOf(PlannedPath other, double vMax, double aMax) {
        return reversedOf(other, vMax, aMax, 0.0, 0.0);
    }

    public static TrajectoryConfig makeConfig(double vMax, double aMax, double vInitial, double vFinal) {
        return new TrajectoryConfig(vMax, aMax)
            .setKinematics(Robot.bot.kinematics)
            .setStartVelocity(vInitial)
            .setEndVelocity(vFinal);
            // .addConstraints(crescendoConstraints());
    }

    private static final List<TrajectoryConstraint> crescendoConstraints() {
        TrajectoryConstraint pickupConstraint = new RectangularRegionConstraint(FieldConfig.getInstance().NOTE_D.plus(new Translation2d(-1, -1)), FieldConfig.getInstance().NOTE_H.plus(new Translation2d(1, 1)), new MaxVelocityConstraint(2.0));
        return List.of(pickupConstraint);
    }

    // make into method
    // private static List<TrajectoryConstraint> chargedUpTrajectoryConstraints = List.of(
    //             // Cable bump
    //             new RectangularRegionConstraint(
    //                 new Translation2d(Community.chargingStationInnerX, Community.rightY),
    //                 new Translation2d(Community.chargingStationOuterX, Community.chargingStationRightY),
    //                 new MaxVelocityConstraint(cableBumpMaxVelocity)),

    //             // Charging station
    //             new RectangularRegionConstraint(
    //                 new Translation2d(
    //                     Community.chargingStationInnerX - 0.8, Community.chargingStationRightY),
    //                 new Translation2d(
    //                     Community.chargingStationOuterX + 0.8, Community.chargingStationLeftY),
    //                 new MaxVelocityConstraint(chargingStationMaxVelocity)));
}
