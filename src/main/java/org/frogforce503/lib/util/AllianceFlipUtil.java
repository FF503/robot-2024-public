package org.frogforce503.lib.util;

import java.util.ArrayList;
import java.util.List;
import java.util.TreeMap;

import org.frogforce503.lib.trajectory.RotationSequence;
import org.frogforce503.lib.trajectory.Waypoint;
import org.frogforce503.lib.trajectory.PlannedPath.HolonomicState;
import org.frogforce503.robot2024.RobotStatus;
import org.frogforce503.robot2024.RobotStatus.AllianceColor;
import org.frogforce503.robot2024.fields.FieldConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;

/**
 * Utility functions for flipping from the blue to red alliance. By default, all translations and
 * poses in {@link FieldConstants} are stored with the origin at the rightmost point on the blue
 * alliance wall.
 */
public class AllianceFlipUtil {

  private static boolean shouldFlipOverride = false;

  /** Flips a translation to the correct side of the field based on the current alliance color. */
  public static Translation2d apply(Translation2d translation) {
    if (shouldFlip()) {
      return new Translation2d(FieldConfig.getInstance().getFieldDimensions().getX() - translation.getX(), translation.getY());
    } else {
      return translation;
    }
  }

  /** Flips an x coordinate to the correct side of the field based on the current alliance color. */
  public static double apply(double xCoordinate) {
    if (shouldFlip()) {
      return FieldConfig.getInstance().getFieldDimensions().getX() - xCoordinate;
    } else {
      return xCoordinate;
    }
  }

  /** Flips a rotation based on the current alliance color. */
  public static Rotation2d apply(Rotation2d rotation) {
    if (shouldFlip()) {
      return new Rotation2d(-rotation.getCos(), rotation.getSin());
    } else {
      return rotation;
    }
  }

  /** Flips a pose to the correct side of the field based on the current alliance color. */
  public static Pose2d apply(Pose2d pose) {
    if (shouldFlip()) {
      return new Pose2d(
          FieldConfig.getInstance().getFieldDimensions().getX() - pose.getX(),
          pose.getY(),
          new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()));
    } else {
      return pose;
    }
  }

  /**
   * Flips a trajectory state to the correct side of the field based on the current alliance color.
   */
  public static Trajectory.State apply(Trajectory.State state) {
    if (shouldFlip()) {
      return new Trajectory.State(
          state.timeSeconds,
          state.velocityMetersPerSecond,
          state.accelerationMetersPerSecondSq,
          new Pose2d(
              FieldConfig.getInstance().getFieldDimensions().getX() - state.poseMeters.getX(),
              state.poseMeters.getY(),
              new Rotation2d(
                  -state.poseMeters.getRotation().getCos(),
                  state.poseMeters.getRotation().getSin())),
          -state.curvatureRadPerMeter);
    } else {
      return state;
    }
  }

  /** Flips a rotation sequence state based on the current alliance color. */
  public static RotationSequence.State apply(RotationSequence.State state) {
    if (shouldFlip()) {
      return new RotationSequence.State(
          new Rotation2d(-state.position.getCos(), state.position.getSin()),
          -state.velocityRadiansPerSec);
    } else {
      return state;
    }
  }

  public static HolonomicState apply(HolonomicState state) {
    if (shouldFlip()) {
        return new HolonomicState(
          new Trajectory.State(
            state.timeSeconds,
            state.velocityMetersPerSecond,
            state.accelerationMetersPerSecondSq,
            new Pose2d(
                FieldConfig.getInstance().getFieldDimensions().getX() - state.poseMeters.getX(),
                state.poseMeters.getY(),
                new Rotation2d(
                    -state.poseMeters.getRotation().getCos(),
                    state.poseMeters.getRotation().getSin())),
            -state.curvatureRadPerMeter
          ),
          new RotationSequence.State(
          new Rotation2d(-state.holonomicAngle.getCos(), state.holonomicAngle.getSin()),
            -state.angularVelocityRadiansPerSec)
        );
    } else {
      return state;
    }
  }

  /** Flips a waypoint */
  public static Waypoint apply(Waypoint waypoint) {
    return new Waypoint(
      new Translation2d(FieldConfig.getInstance().getFieldDimensions().getX() - waypoint.getTranslation().getX(), waypoint.getTranslation().getY()), 
      waypoint.getDriveRotation().isPresent() ? new Rotation2d(-waypoint.getDriveRotation().get().getCos(), waypoint.getDriveRotation().get().getSin()) : null, 
      waypoint.getHolonomicRotation().isPresent() ?  new Rotation2d(-waypoint.getHolonomicRotation().get().getCos(), waypoint.getHolonomicRotation().get().getSin()) : null
    ).tagged(waypoint.getTag());
  }

  public static Trajectory apply(Trajectory traj) {
    List<Trajectory.State> states = new ArrayList<>();
    for (Trajectory.State state : traj.getStates()) {
      states.add(apply(state));
    }
    return new Trajectory(states);
  }

  public static RotationSequence apply(RotationSequence rotationSequence) {
    TreeMap<Double, Rotation2d> sequence = rotationSequence.getTreeMap();

    for (Double d : sequence.keySet()) {
      sequence.put(d, apply(sequence.get(d)));
    }

    return new RotationSequence(sequence);

    // List<RotationSequence.State> states = new ArrayList<>();
    // for (RotationSequence.State state : rotationSequence.getStates()) {
    //   states.add(apply(state));
    // }
    // return new RotationSequence(states);
  }

  public static void override() {
    shouldFlipOverride = true;
  }

  private static boolean shouldFlip() {
    if (shouldFlipOverride) {
      shouldFlipOverride = false;
      return true;
    }
    return RobotStatus.getInstance().getAllianceColor() == AllianceColor.RED;
  }
}