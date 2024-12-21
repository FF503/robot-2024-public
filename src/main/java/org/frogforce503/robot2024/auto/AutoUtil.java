package org.frogforce503.robot2024.auto;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.stream.Stream;

import org.frogforce503.lib.auto.AutoChooser.StartingLocation;
import org.frogforce503.lib.trajectory.PlannedPath;
import org.frogforce503.lib.trajectory.Waypoint;
import org.frogforce503.lib.util.AllianceFlipUtil;
import org.frogforce503.robot2024.RobotStatus;
import org.frogforce503.robot2024.RobotStatus.AllianceColor;
import org.frogforce503.robot2024.fields.FieldConfig;
import org.frogforce503.robot2024.planners.ShotPlanner;

import com.fasterxml.jackson.core.StreamWriteCapability;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

// contains commonly used functions and actions
public class AutoUtil {

    public static boolean onFirstShot = false;
    public static boolean isOnCable = false;
    public static boolean useFrontOnly = false;

    public static boolean frontCam = true;

    public static PlannedPath[] paths(PlannedPath[] ...paths) {
        return Stream.of(paths).flatMap(Stream::of)
                      .toArray(PlannedPath[]::new);
    }

    public static class Waypoints {

        public static Pose2d facingGoal(Pose2d pose) {
            return new Pose2d(pose.getTranslation(), (ShotPlanner.getInstance().getRawAngleToGoal(pose.getTranslation()).plus(new Rotation2d(Math.PI))));
        }

        public static Pose2d facingGoal(Translation2d translation) {
            return facingGoal(new Pose2d(translation, new Rotation2d()));
        }

        public static Pose2d flipHeading(Pose2d existing) {
            return new Pose2d(existing.getTranslation(), existing.getRotation().unaryMinus());
        }

        public static List<Waypoint> shift(List<Waypoint> input, Translation2d delta) {
            ArrayList<Waypoint> output = new ArrayList<>();

            for (Waypoint w : input) {
                output.add(w.plus(delta));
            }

            return output;
        }

        public static List<Waypoint> withHeading(List<Waypoint> input, Rotation2d heading) {
            ArrayList<Waypoint> output = new ArrayList<>();

            for (Waypoint w : input) {
                output.add(w.setHolonomicRotation(heading));
            }

            return output;
        }

        public static List<Waypoint> climbingCrossover(Translation2d start) {
            // from inside center grid is inferred
            // Translation2d parkTarget = climbingPoints(true, StartingLocation.CENTER_GRID).get(2).getTranslation();

            
            Waypoint past = new Waypoint(start.plus(new Translation2d(1.7 - Units.feetToMeters(1.0) + Units.feetToMeters(2.0), 0)), null, null);
            Waypoint comeBack = new Waypoint(start.plus(new Translation2d(-0.4, 0)), null, null);

            return List.of(past, comeBack);
        }

        // public static RectangularRegionConstraint cableProtectorRegion() {
        //     Translation2d bottomLeft = 

        //     return new RectangularRegionConstraint(, null, new MaxVelocityConstraint(3.0))
        // }

        public static Translation2d translation2d() {
            return translation(0, 0);
        }

        public static Translation2d translation(double x, double y) {
            return (new Translation2d(RobotStatus.getInstance().getAllianceColor() == AllianceColor.RED ? x : x, y));
        }

        public static Translation2d translation(double d, Rotation2d angle) {
            return (new Translation2d(d, rotation(angle.getDegrees())));
        }

        public static Rotation2d rotation(double degrees) {
            return Rotation2d.fromDegrees(degrees);
        }

        public static Rotation2d rotation() {
            return rotation(0);
        }

    }    
}
