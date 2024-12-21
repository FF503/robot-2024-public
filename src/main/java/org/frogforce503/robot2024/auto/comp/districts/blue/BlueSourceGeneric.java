package org.frogforce503.robot2024.auto.comp.districts.blue;

import java.util.HashMap;

import org.frogforce503.lib.auto.AutoMode;
import org.frogforce503.lib.auto.AutoChooser.CENTERLINE_NOTES;
import org.frogforce503.lib.trajectory.PlannedPath;
import org.frogforce503.lib.trajectory.Route;
import org.frogforce503.lib.trajectory.SwervePathBuilder;
import org.frogforce503.lib.trajectory.Waypoint;
import org.frogforce503.robot2024.RobotContainer;
import org.frogforce503.robot2024.auto.AutoUtil.Waypoints;
import org.frogforce503.robot2024.fields.FieldConfig;
import org.frogforce503.robot2024.planners.ShotPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class BlueSourceGeneric extends AutoMode {

    private HashMap<CENTERLINE_NOTES, PlannedPath> grabFirstPath = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> grabSecondPath = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> scorePath = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> hailMaryPath = new HashMap<>();

    private PlannedPath goOutAndShootFirst;

    private Pose2d shootingPose;
    private double firstHint;

    private CENTERLINE_NOTES note1, note2, note3;
    public BlueSourceGeneric(CENTERLINE_NOTES note1, CENTERLINE_NOTES note2, CENTERLINE_NOTES note3) {
        this.note1 = note1;
        this.note2 = note2;
        this.note3 = note3;

        var startingPose = setupPose(Waypoints.facingGoal(new Translation2d(FieldConfig.getInstance().BLUE_INITIATION_LINE - 0.5, FieldConfig.getInstance().NOTE_I.getY())));

        var shootingPosition = FieldConfig.getInstance().BLUE_STAGE_RIGHT.interpolate(FieldConfig.getInstance().BLUE_CENTER_STAGE, 0.5)
            .plus(new Translation2d(Units.feetToMeters(-5), Units.feetToMeters(-4)));
        shootingPose = Waypoints.facingGoal(shootingPosition);

        firstHint = ShotPlanner.getInstance().getRawDistanceToGoal(shootingPosition);

        goOutAndShootFirst = SwervePathBuilder.generate(4.5, 3.75, 0, 0.375,
            Waypoint.fromHolonomicPose(startingPose),
            Waypoint.fromDifferentialPose(shootingPose)
        );

        var grabDFirst = SwervePathBuilder.generate(4.6, 3.75, 0.375, 0,
            Waypoint.fromDifferentialPose(shootingPose),
            new Waypoint(shootingPosition.plus(new Translation2d(Units.feetToMeters(2), shootingPose.getRotation())), shootingPose.getRotation(), shootingPose.getRotation()),
            new Waypoint(FieldConfig.getInstance().NOTE_D.plus(new Translation2d(Units.feetToMeters(0.25), Units.feetToMeters(0.35))), new Rotation2d(), new Rotation2d())
        );

        var grabEFirst = SwervePathBuilder.generate(4.6, 3.75, 0.375, 0,
            Waypoint.fromDifferentialPose(shootingPose),
            new Waypoint(shootingPosition.plus(new Translation2d(Units.feetToMeters(1), shootingPose.getRotation())), shootingPose.getRotation(), shootingPose.getRotation()),
            new Waypoint(FieldConfig.getInstance().NOTE_E.plus(new Translation2d(Units.feetToMeters(0), Units.feetToMeters(1.25))), new Rotation2d(), new Rotation2d())
        );

        var grabFFirst = SwervePathBuilder.generate(4.6, 3.75, 0.375, 0,
            Waypoint.fromDifferentialPose(shootingPose),
            new Waypoint(shootingPosition.plus(new Translation2d(Units.feetToMeters(1), shootingPose.getRotation())), shootingPose.getRotation(), shootingPose.getRotation()),
            new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(Units.feetToMeters(-0.25), 0)), null, new Rotation2d(Math.PI/4))
        );

        grabFirstPath.put(CENTERLINE_NOTES.D, grabDFirst);
        grabFirstPath.put(CENTERLINE_NOTES.E, grabEFirst);
        grabFirstPath.put(CENTERLINE_NOTES.F, grabFFirst);

        var scoreD = SwervePathBuilder.generate(4.6, 3.75, 0.0, 0,
            Waypoint.fromHolonomicPose(grabDFirst.getFinalHolonomicPose()),
            Waypoint.fromHolonomicPose(shootingPose)
        );

        var scoreE = SwervePathBuilder.generate(4.6, 3.75, 0.0, 0,
            Waypoint.fromHolonomicPose(grabEFirst.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_RIGHT.plus(new Translation2d(0, -1.25))),
            Waypoint.fromHolonomicPose(shootingPose)
        );

        var scoreF = SwervePathBuilder.generate(4.6, 3.75, 0.0, 0,
            Waypoint.fromHolonomicPose(grabFFirst.getFinalHolonomicPose()).setHolonomicRotation(new Rotation2d()),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_RIGHT.interpolate(FieldConfig.getInstance().BLUE_STAGE_LEFT, 0.5).interpolate(FieldConfig.getInstance().BLUE_CENTER_STAGE, 0.125), new Rotation2d(Math.PI), new Rotation2d()),
            Waypoint.fromHolonomicPose(shootingPose)
        );

        scorePath.put(CENTERLINE_NOTES.D, scoreD);
        scorePath.put(CENTERLINE_NOTES.E, scoreE);
        scorePath.put(CENTERLINE_NOTES.F, scoreF);

        grabSecondPath.put(CENTERLINE_NOTES.D, SwervePathBuilder.regenerate(4.6, 3.75, 0.0, 0.0, grabDFirst));
        grabSecondPath.put(CENTERLINE_NOTES.E, SwervePathBuilder.regenerate(4.6, 3.75, 0.0, 0.0, grabEFirst));
        grabSecondPath.put(CENTERLINE_NOTES.F, SwervePathBuilder.reversedOf(scoreF, 4.6, 3.75, 0.0, 0.0));
    }

    @Override
    public CENTERLINE_NOTES[] priorityList() {
        return new CENTERLINE_NOTES[] {note1, note2, note3};
    }

    @Override
    public Command routine() {
        return Commands.sequence(
            RobotContainer.shooter.rampUp(),
            Commands.deadline( // grabs G while shooting P
                Commands.sequence(
                    drive(goOutAndShootFirst, () -> true),
                    drive(grabFirstPath.get(note1))
                ),
                Commands.waitUntil(() -> RobotContainer.drive.getPose().getTranslation().getX() >= shootingPose.getX() - Units.feetToMeters(0.5))
                    .deadlineWith(RobotContainer.hint(firstHint + Units.feetToMeters(12)))
                    .andThen(
                        RobotContainer.shootSequence(),
                        RobotContainer.intake().alongWith(RobotContainer.wrist.intakeAssist())
                    )
            ),
            driveAndIntake(scorePath.get(note1), () -> true),
            RobotContainer.shootSequence(),
            driveAndIntake(grabSecondPath.get(note2)),
            driveAndIntake(scorePath.get(note2), () -> true),
            RobotContainer.shootSequence(),
            drive(grabSecondPath.get(note3)).alongWith(RobotContainer.intake())// let intaking actually finish
            // RobotContainer.hailMary()
            // driveAndIntake(scorePath.get(note3), () -> true),
            // RobotContainer.shootSequence()
        );
    }

    @Override
    public Route getRoute() {
        return new Route()
            .addPaths(goOutAndShootFirst)
            .addPathOptions(grabFirstPath)
            .addPathOptions(scorePath)
            .addPathOptions(grabSecondPath);
    }
    
}
