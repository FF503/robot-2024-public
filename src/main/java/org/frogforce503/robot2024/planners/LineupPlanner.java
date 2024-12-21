package org.frogforce503.robot2024.planners;

import java.util.Arrays;
import java.util.function.Supplier;

import org.frogforce503.lib.auto.follower.SwerveFollowPathCommand;
import org.frogforce503.lib.trajectory.PlannedPath;
import org.frogforce503.lib.trajectory.SwervePathBuilder;
import org.frogforce503.lib.trajectory.Waypoint;
import org.frogforce503.robot2024.RobotContainer;
import org.frogforce503.robot2024.auto.AutoUtil.Waypoints;
import org.frogforce503.robot2024.fields.FieldConfig;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class LineupPlanner {
    public enum LineupGoal {
        CENTER_CLIMB(() -> {
            Pose2d tag = FieldConfig.getInstance().getTagById(RobotContainer.red() ? 13 : 14);
            Translation2d lineup = LineupPlanner.getClimbOffset(tag);
            return new Pose2d(lineup, tag.getRotation());
        }), 
        LEFT_CLIMB(() -> {
            Pose2d tag = FieldConfig.getInstance().getTagById(RobotContainer.red() ? 11 : 15);
            Translation2d lineup = LineupPlanner.getClimbOffset(tag).plus(new Translation2d(Units.feetToMeters(1), tag.getRotation().plus(new Rotation2d(Math.PI/2))));
            return new Pose2d(lineup, tag.getRotation());
        }),
        RIGHT_CLIMB(() -> {
            Pose2d tag = FieldConfig.getInstance().getTagById(RobotContainer.red() ? 12 : 16);
            Translation2d lineup = LineupPlanner.getClimbOffset(tag);
            return new Pose2d(lineup, tag.getRotation());
        }),
        AMP(() -> {
            Pose2d tag = FieldConfig.getInstance().getTagById(RobotContainer.red() ? 5 : 6);
            Translation2d lineup = tag.getTranslation().minus(new Translation2d(0.0, Units.feetToMeters(1.0)));
            return new Pose2d(lineup, tag.getRotation());
        }),
        FRONT_BATTER(() -> {
            Pose2d tag = FieldConfig.getInstance().getTagById(RobotContainer.red() ? 4 : 7);
            Translation2d lineup = tag.getTranslation().minus(new Translation2d(Units.feetToMeters(RobotContainer.red() ? 5.0 : -5.0), 0.0));
            return new Pose2d(lineup, tag.getRotation().plus(new Rotation2d(Math.PI)));
        }),
        NONE(() -> new Pose2d());
        
        public Supplier<Pose2d> target;

        LineupGoal(Supplier<Pose2d> target) {
            this.target = target;
        }
    }

    private static LineupPlanner instance = null;
    
    public static LineupPlanner getInstance() {
        if (instance == null) { instance = new LineupPlanner(); }
        return instance;
    }

    public Command driveTo(LineupGoal goal) {
        return RobotContainer.drive.driveToPoint(goal.target.get());
    }

    public Command driveTo(LineupMarker waypoint) {
        return RobotContainer.drive.driveToPoint(waypoint.getPose());
    }

    public Command aimTo(LineupGoal goal) {
        return RobotContainer.drive.snapToAngle(goal.target.get().getRotation());
    }

    public Command aimTo(LineupMarker waypoint) {
        return RobotContainer.drive.snapToAngle(waypoint.getRotation());
    }

    public Command lineupTo(LineupGoal goal) {
        return Commands.sequence(
            driveTo(goal),   
            aimTo(goal)
        );
        // .unless( // doesn't drive / aim without cameras (on real robot)
        //     Logic.and(
        //         () -> RobotBase.isReal(),
        //         () -> (RobotContainer.photon.maxTagsFromOneCam() <= 0)
        //     )
        // );

        // had to comment out the unless statement as cameras don't see anything for a little time (~0.1 seconds), doesn't mean robot has to stop
    }

    public Command lineupTo(LineupMarker waypoint) {
        return Commands.sequence(
            driveTo(waypoint),   
            aimTo(waypoint)
        );
        // .unless( // doesn't drive / aim without cameras (on real robot)
        //     Logic.and(
        //         () -> RobotBase.isReal(),
        //         () -> (RobotContainer.photon.maxTagsFromOneCam() <= 0)
        //     )
        // );

        // had to comment out the unless statement as cameras don't see anything for a little time (~0.1 seconds), doesn't mean robot has to stop
    }

    public Command drivePath(LineupMarker... waypoints) {
        return Commands.sequence(
            Arrays.stream(waypoints)
                .map(waypoint -> RobotContainer.drive.driveToPoint(waypoint.getPose(), waypoint.getCruiseVel()))
                .toArray(Command[]::new)
        );
    }

    // NOTE_F -> MIDDLE OF BLUE LEFT & RIGHT TRUSS -> BlueSource4NoteOFSZ_SHIFT shotPose -> BLUE AMP
    public Command pathing_DOP() {
        Pose2d shotPose = Waypoints.facingGoal(
            FieldConfig.getInstance().BLUE_STAGE_LEFT.plus(new Translation2d(-1.5, -0.5))
        );

        return Commands.sequence(
            drivePath(
                new LineupMarker(FieldConfig.getInstance().NOTE_F, shotPose.getRotation()),
                new LineupMarker(FieldConfig.getInstance().BLUE_STAGE_RIGHT.interpolate(FieldConfig.getInstance().BLUE_STAGE_LEFT, 0.5), shotPose.getRotation(), 3.0),
                new LineupMarker(shotPose, 3.0)
            ),
            lineupTo(LineupGoal.AMP)
        );
    }

    /**
     * Similar to Waypoint.java class, but has an additional attribute called cruiseVel (required for driveToPoint as it offers smoothness when switching between points)
     */
    public class LineupMarker {
        private Pose2d pose;
        private double cruiseVel;

        public LineupMarker(Pose2d pose, double cruiseVel) {
            this.pose = pose;
            this.cruiseVel = cruiseVel;
        }

        public LineupMarker(Pose2d pose) {
            this(pose, 0.0);
        }

        public LineupMarker(Translation2d translation, Rotation2d rotation, double cruiseVel) {
            this(new Pose2d(translation, rotation), cruiseVel);
        }

        public LineupMarker(Translation2d translation, Rotation2d rotation) {
            this(translation, rotation, 0.0);
        }

        public LineupMarker() {
            this(new Pose2d());
        }

        public Translation2d getTranslation() {
            return this.pose.getTranslation();
        }

        public Rotation2d getRotation() {
            return this.pose.getRotation();
        }

        public Pose2d getPose() {
            return this.pose;
        }

        public double getCruiseVel() {
            return this.cruiseVel;
        }

        public LineupMarker withTranslation(Translation2d newTranslation) {
            this.pose = new Pose2d(newTranslation, this.pose.getRotation());
            return this;
        }

        public LineupMarker withRotation(Rotation2d newHeading) {
            this.pose = new Pose2d(this.pose.getTranslation(), newHeading);
            return this;
        }

        public LineupMarker withCruiseVel(double newCruiseVel) {
            this.cruiseVel = newCruiseVel;
            return this;
        }
    }

    

    // ------------------------ SWERVE PATH BUILDER LINEUP METHODS (take a little time to generate correctly) ------------------------ //

    public Command lineupPlanner_TEST() {
        Pose2d test = LineupGoal.AMP.target.get();

        try {
            // Translation2d x = new Translation2d();
            // if (RobotContainer.drive.getPose().getY() >= FieldConfig.getInstance().BLUE_STAGE_LEFT.getY()) {
            //     x = FieldConfig.getInstance().BLUE_STAGE_LEFT.plus(new Translation2d(-1.5, 0.0));
            // } else if (RobotContainer.drive.getPose().getY() >= FieldConfig.getInstance().BLUE_STAGE_RIGHT.getY()) {
            //     x = FieldConfig.getInstance().BLUE_CENTER_STAGE.plus(new Translation2d(1.5, 0.0));
            // } else if (RobotContainer.drive.getPose().getY() < FieldConfig.getInstance().BLUE_STAGE_RIGHT.getY()) {
            //     x = FieldConfig.getInstance().BLUE_STAGE_RIGHT.plus(new Translation2d(-1.5, 0.0));
            // }

            PlannedPath path = SwervePathBuilder.generate(2.0, 3.0,
                Waypoint.fromHolonomicPose(RobotContainer.drive.getPose()),
                Waypoint.fromHolonomicPose(test)
            );

            return new SwerveFollowPathCommand(path);
        } catch (Exception e) {
            System.out.println("CAN'T GENERATE PATH ERROR");
            e.printStackTrace();
            return Commands.print("CAN'T GENERATE PATH ERROR");
        }
    }

    private static Translation2d getClimbOffset(Pose2d tagPose) {
        Rotation2d normal = tagPose.getRotation();
        double LINEUP_OFFSET = Units.feetToMeters(1.25);
        
        return tagPose.getTranslation().plus(new Translation2d(LINEUP_OFFSET, normal));
    }

    private Pair<Pose2d, Double> closestTo(Pose2d target, Pose2d ...options) {
        double closestDist = Double.MAX_VALUE;
        Pose2d closest = new Pose2d();

        for (Pose2d pose : options) {
            var dist = target.minus(pose).getTranslation().getNorm();
            if (dist <= closestDist) {
                closest = pose;
                closestDist = dist;
            }
        }

        return Pair.of(closest, closestDist);
    }

    public Command navigateToAmp() {
        Pose2d ampPose = LineupGoal.AMP.target.get();

        try {
            PlannedPath path = SwervePathBuilder.generate(1, 1,
                Waypoint.fromHolonomicPose(RobotContainer.drive.getPose()),
                Waypoint.fromHolonomicPose(ampPose)
            );

            return new SwerveFollowPathCommand(path);
        } catch (Exception e) {
            System.out.println("CAN'T GENERATE PATH --- TOO CLOSE");
            e.printStackTrace();
            return Commands.none();
        }
    }

    public Command navigateToGoal() {
        Pair<Pose2d, Double> target = closestTo(
            RobotContainer.drive.getPose(), 
            LineupGoal.LEFT_CLIMB.target.get(), 
            LineupGoal.CENTER_CLIMB.target.get(), 
            LineupGoal.RIGHT_CLIMB.target.get()
        );

        if (target.getSecond() > Units.feetToMeters(5)) {
            System.out.println("-------TOO FAR--------");
            return Commands.none();
        }
        
        try {
            PlannedPath path = SwervePathBuilder.generate(1, 1,
                Waypoint.fromHolonomicPose(RobotContainer.drive.getPose()),
                Waypoint.fromHolonomicPose(target.getFirst())
            );

            return new SwerveFollowPathCommand(path);
        } catch (Exception e) {
            System.out.println("-------CANT GENERATE THE PATH (TOO CLOSE)-------");
            e.printStackTrace();
            return Commands.none();
        }
    }
}