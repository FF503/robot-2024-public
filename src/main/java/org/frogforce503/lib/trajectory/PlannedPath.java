package org.frogforce503.lib.trajectory;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;

public class PlannedPath {
    private Trajectory driveTrajectory;
    private RotationSequence rotationSequence;
    private List<Waypoint> waypoints; 

    public PlannedPath(Trajectory driveTrajectory, RotationSequence rotationSequence, List<Waypoint> waypoints) {
        this.driveTrajectory = driveTrajectory;
        this.rotationSequence = rotationSequence;
        this.waypoints = waypoints;
    }

    public Trajectory getDriveTrajectory() {
        return this.driveTrajectory;
    }

    public RotationSequence getRotationSequence() {
        return this.rotationSequence;
    }

    public List<Waypoint> getWaypoints() {
        return this.waypoints;
    }

    public Pose2d getInitialHolonomicPose() {
        Pair<Trajectory.State, RotationSequence.State> res = _sample(0);
        return (new Pose2d(res.getFirst().poseMeters.getTranslation(), res.getSecond().position));
    }

    public Pose2d getFinalHolonomicPose() {
        Pair<Trajectory.State, RotationSequence.State> res = _sample(getTotalTimeSeconds());
        return (new Pose2d(res.getFirst().poseMeters.getTranslation(), res.getSecond().position));    
    }

    public Waypoint getFinalWaypoint() {
        return this.waypoints.get(this.waypoints.size() - 1);
    }

    public List<Waypoint> sampleTimeRange(double ti, double tf, double intervals) {
        List<Waypoint> res = new ArrayList<Waypoint>();

        for (double i = 0; i <= intervals; i++) {
            res.add(
                Waypoint.fromPlannedPathState(
                    this.sample(MathUtil.interpolate(ti, tf, i * (1.0 / intervals)))   
                )
            );
        }

        return res;
    }

    public List<Waypoint> sampleTimeRange(double ti, double tf) {
        return sampleTimeRange(ti, tf, 10.0); // 10 intervals is accurate enough to match original path
    }

    private Pair<Trajectory.State, RotationSequence.State> _sample(double timeSeconds) {
        return Pair.of((this.driveTrajectory.sample(timeSeconds)), (this.rotationSequence.sample(timeSeconds)));
    }

    public HolonomicState sample(double timeSeconds) {
        Pair<Trajectory.State, RotationSequence.State> pair = this._sample(timeSeconds);
        return new HolonomicState(pair.getFirst(), pair.getSecond());
    }

    public double getTotalTimeSeconds() {
        return getDriveTrajectory().getTotalTimeSeconds();
    }

    public static class HolonomicState {
        // The time elapsed since the beginning of the trajectory.
        public double timeSeconds;

        // The speed at that point of the trajectory.
        public double velocityMetersPerSecond;

        // The acceleration at that point of the trajectory.
        public double accelerationMetersPerSecondSq;

        // The pose at that point of the trajectory.
        public Pose2d poseMeters;

        // The curvature at that point of the trajectory.
        public double curvatureRadPerMeter;

        public Rotation2d holonomicAngle; // which direction the robot is facing
        public double angularVelocityRadiansPerSec; // speed at which the robot is rotating

        public HolonomicState(Trajectory.State trajState, RotationSequence.State rotState) {
            this.timeSeconds = trajState.timeSeconds;
            this.velocityMetersPerSecond = trajState.velocityMetersPerSecond;
            this.accelerationMetersPerSecondSq = trajState.accelerationMetersPerSecondSq;
            this.poseMeters = trajState.poseMeters;//new Pose2d(trajState.poseMeters.getTranslation(), rotState.position);
            this.curvatureRadPerMeter = trajState.curvatureRadPerMeter;
            this.holonomicAngle = rotState.position;
            this.angularVelocityRadiansPerSec = rotState.velocityRadiansPerSec;
        }
    }
    
}