package org.frogforce503.lib.trajectory;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Optional;

import org.frogforce503.lib.auto.AutoChooser.CENTERLINE_NOTES;
import org.frogforce503.robot2024.subsystems.NoteDetector.SIDE;

/** A trajectory waypoint, including a translation and optional drive/holonomic rotations. */
public class Waypoint {
  private final Translation2d translation;
  private Rotation2d driveRotation;
  private Rotation2d holonomicRotation;
  private Tag tag = null;

  /** Constructs a Waypoint at the origin and without a drive or holonomic rotation. */
  public Waypoint() {
    this(new Translation2d());
  }

  /**
   * Constructs a Waypoint with a translation, drive rotation, and holonomic rotation.
   *
   * @param translation Waypoint position (required)
   * @param driveRotation Drive velocity rotation (optional, can be null)
   * @param holonomicRotation Holonomic rotation (optional, can be null)
   */
  public Waypoint(
      Translation2d translation, Rotation2d driveRotation, Rotation2d holonomicRotation) {
    this.translation = requireNonNullParam(translation, "translation", "Waypoint");
    this.driveRotation = driveRotation;
    this.holonomicRotation = holonomicRotation;
  }

  /**
   * Constructs a Waypoint with a translation (but no drive or holonomic rotation).
   *
   * @param translation Waypoint position (required)
   */
  public Waypoint(Translation2d translation) {
    this.translation = requireNonNullParam(translation, "translation", "Waypoint");
    this.driveRotation = null;
    this.holonomicRotation = null;
  }

  public Waypoint tagged(Tag tag) {
    this.tag = tag;
    return this;
  }

  public Tag getTag() {
    return this.tag;
  }

  /**
   * Constucts a Waypoint based on a pose.
   *
   * @param pose Source pose (where the rotation describes the drive rotation)
   */
  public static Waypoint fromDifferentialPose(Pose2d pose) {
    requireNonNullParam(pose, "pose", "Waypoint");
    return new Waypoint(pose.getTranslation(), pose.getRotation(), null);
  }

  public Waypoint invertDriveRotation() {
    return new Waypoint(this.translation, this.driveRotation.rotateBy(Rotation2d.fromDegrees(180)), this.holonomicRotation);
  }

  /**
   * Constucts a Waypoint based on a pose.
   *
   * @param pose Source pose (where the rotation describes the drive rotation)
   * @param holonomicRotation Holonomic rotation
   */
  public static Waypoint fromDifferentialPose(Pose2d pose, Rotation2d holonomicRotation) {
    requireNonNullParam(pose, "pose", "Waypoint");
    return new Waypoint(pose.getTranslation(), pose.getRotation(), holonomicRotation);
  }

  /**
   * Constucts a Waypoint based on a pose.
   *
   * @param pose Source pose (where the rotation describes the holonomic rotation)
   */
  public static Waypoint fromHolonomicPose(Pose2d pose) {
    requireNonNullParam(pose, "pose", "Waypoint");
    return new Waypoint(pose.getTranslation(), null, pose.getRotation());
  }

  /**
   * Constucts a Waypoint based on a pose.
   *
   * @param pose Source pose (where the rotation describes the holonomic rotation)
   * @param driveRotation Drive rotation
   */
  public static Waypoint fromHolonomicPose(Pose2d pose, Rotation2d driveRotation) {
    requireNonNullParam(pose, "pose", "Waypoint");
    return new Waypoint(pose.getTranslation(), driveRotation, pose.getRotation());
  }
  
  public static Waypoint fromPlannedPathState(PlannedPath.HolonomicState state) {
    return new Waypoint(state.poseMeters.getTranslation(), null, state.holonomicAngle);
  }

  public Waypoint withDriveRotation(Rotation2d newHeading) {
    this.driveRotation = newHeading;
    return this;
  }

  public Waypoint withHolonomicRotation(Rotation2d newHeading) {
    this.holonomicRotation = newHeading;
    return this;
  }

  /** Returns the translation component of the waypoint. */
  public Translation2d getTranslation() {
    return translation;
  }

  /**
   * Returns the drive rotation component of the waypoint (or an empty optional if not specified).
   */
  public Optional<Rotation2d> getDriveRotation() {
    return Optional.ofNullable(driveRotation);
  }

  /**
   * Returns the holonomic rotation component of the waypoint (or an empty optional if not
   * specified).
   */
  public Optional<Rotation2d> getHolonomicRotation() {
    return Optional.ofNullable(holonomicRotation);
  }

  public Waypoint plus(Translation2d t) {
    return new Waypoint(this.translation.plus(t), driveRotation, holonomicRotation);
  }

  public Waypoint setHolonomicRotation(Rotation2d r) {
    return new Waypoint(translation, driveRotation, r);
  }

  public Waypoint setDriveRotationRotation(Rotation2d r) {
    return new Waypoint(translation, r, holonomicRotation);
  }

  public static class Tag {
    private double timestamp = -1;
    private boolean passed = false;
    private SIDE choice = SIDE.NONE;
    private CENTERLINE_NOTES note = CENTERLINE_NOTES.NONE;

    public void pass(boolean p) {
      if (p)
        this.passed = true;
    }
    
    public void stamp(double s) {
      this.timestamp = s;
    }

    public void choose(SIDE side) {
      this.choice = side;
    }

    public Tag select(CENTERLINE_NOTES note) {
      this.note = note;
      return this;
    }
    
    public void resetChoice() {
      this.choose(SIDE.NONE);
    }

    public double getStamp() {
      return this.timestamp;
    }
    
    public boolean passed() {
      return this.passed;
    }

    public SIDE getChoice() {
      return this.choice;
    }

    public CENTERLINE_NOTES getNote() {
      return this.note;
    }
  }
}