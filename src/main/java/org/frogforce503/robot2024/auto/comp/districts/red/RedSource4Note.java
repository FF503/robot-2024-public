package org.frogforce503.robot2024.auto.comp.districts.red;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import java.util.HashMap;
import java.util.Map;

import org.frogforce503.lib.auto.AutoMode;
import org.frogforce503.lib.auto.AutoChooser.CENTERLINE_NOTES;
import org.frogforce503.lib.trajectory.PlannedPath;
import org.frogforce503.lib.trajectory.Route;
import org.frogforce503.lib.trajectory.SwervePathBuilder;
import org.frogforce503.lib.trajectory.Waypoint;
import org.frogforce503.lib.trajectory.Waypoint.Tag;
import org.frogforce503.lib.util.Logic;
import org.frogforce503.robot2024.RobotContainer;
import org.frogforce503.robot2024.auto.AutoUtil.Waypoints;
import org.frogforce503.robot2024.fields.FieldConfig;
import org.frogforce503.robot2024.planners.ShotPlanner;
import org.frogforce503.robot2024.planners.ShotPlanner.Presets;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RedSource4Note extends AutoMode {

    private HashMap<CENTERLINE_NOTES, PlannedPath> grabFirstPath = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> grabSecondPath = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> bailFromPath = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> scorePath = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> hailMaryPath = new HashMap<>();

    private PlannedPath goOutAndShootFirst;

    private Pose2d shootingPose;
    private double firstHint;

    private CENTERLINE_NOTES note1, note2, note3;

    private double bailTime = 1.125; //s

    public RedSource4Note(CENTERLINE_NOTES note1, CENTERLINE_NOTES note2, CENTERLINE_NOTES note3) {
        this.note1 = note1;
        this.note2 = note2;
        this.note3 = note3;

        var startingPose = setupPose(Waypoints.facingGoal(new Translation2d(FieldConfig.getInstance().RED_INITIATION_LINE + 0.5, FieldConfig.getInstance().NOTE_A.getY())));

        var shootingPosition = FieldConfig.getInstance().RED_STAGE_LEFT.plus(new Translation2d(1, -1));
        shootingPose = Waypoints.facingGoal(shootingPosition);

        firstHint = ShotPlanner.getInstance().getRawDistanceToGoal(shootingPosition);

        // goOutAndShootFirst = SwervePathBuilder.generate(4.5, 3.75, 0, 0.375,
        //     Waypoint.fromHolonomicPose(startingPose),
        //     Waypoint.fromDifferentialPose(shootingPose)
        // );

        var grabDFirst = SwervePathBuilder.generate(5.5, 4.5, 0.0, 0,
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(shootingPosition.plus(new Translation2d(Units.feetToMeters(2), shootingPose.getRotation())), shootingPose.getRotation(), null),
            new Waypoint(FieldConfig.getInstance().NOTE_D.plus(new Translation2d(Units.feetToMeters(0.1), Units.feetToMeters(-0.5))), new Rotation2d(Math.PI), new Rotation2d(Math.PI))
        );

        var grabEFirst = SwervePathBuilder.generate(5, 4.25, 0.0, 0,
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(shootingPosition.plus(new Translation2d(Units.feetToMeters(1), shootingPose.getRotation())), shootingPose.getRotation(), shootingPose.getRotation()),
            new Waypoint(FieldConfig.getInstance().NOTE_E.plus(new Translation2d(-Units.feetToMeters(0.0), Units.inchesToMeters(0))), new Rotation2d(Math.PI), new Rotation2d(Math.PI))
        );

        var grabFFirst = SwervePathBuilder.generate(5, 4.25, 0.0, 0,
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(shootingPosition.plus(new Translation2d(Units.feetToMeters(1), shootingPose.getRotation())), shootingPose.getRotation(), shootingPose.getRotation()),
            new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(Units.feetToMeters(0.25), 0)), null, new Rotation2d(3 * Math.PI/4))
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
            new Waypoint(FieldConfig.getInstance().RED_STAGE_LEFT.plus(new Translation2d(0, -1.25))),
            Waypoint.fromHolonomicPose(shootingPose)
        );

        var scoreF = SwervePathBuilder.generate(4.6, 3.75, 0.0, 0,
            Waypoint.fromHolonomicPose(grabFFirst.getFinalHolonomicPose()).setHolonomicRotation(new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_LEFT.interpolate(FieldConfig.getInstance().RED_STAGE_RIGHT, 0.35).interpolate(FieldConfig.getInstance().RED_CENTER_STAGE, 0.5), new Rotation2d(), new Rotation2d(Math.PI)),
            Waypoint.fromHolonomicPose(shootingPose)
        );

        scorePath.put(CENTERLINE_NOTES.D, scoreD);
        scorePath.put(CENTERLINE_NOTES.E, scoreE);
        scorePath.put(CENTERLINE_NOTES.F, scoreF);

        grabSecondPath.put(CENTERLINE_NOTES.D, SwervePathBuilder.reversedOf(scoreD, 4.6, 3.75, 0.0, 0.0));
        grabSecondPath.put(CENTERLINE_NOTES.E, SwervePathBuilder.reversedOf(scoreE, 4.6, 3.75, 0.0, 0.0));
        grabSecondPath.put(CENTERLINE_NOTES.F, SwervePathBuilder.reversedOf(scoreF, 4.6, 3.75, 0.0, 0.0));

        var bailFromDToE = SwervePathBuilder.generate(4.0, 3.5,
            Waypoint.fromPlannedPathState(scoreD.sample(bailTime)),
            new Waypoint(grabEFirst.getFinalHolonomicPose().getTranslation(), new Rotation2d(Math.PI), Rotation2d.fromDegrees(135))
        );
        
        var bailFromEtoF = SwervePathBuilder.generate(4.0, 3.5,
            Waypoint.fromPlannedPathState(scoreE.sample(bailTime)),
            Waypoint.fromDifferentialPose(grabFFirst.getFinalHolonomicPose())
        );

        bailFromPath.put(CENTERLINE_NOTES.D, bailFromDToE);
        bailFromPath.put(CENTERLINE_NOTES.E, bailFromEtoF);
    }

    @Override
    public CENTERLINE_NOTES[] priorityList() {
        return new CENTERLINE_NOTES[] {note1, note2, note3};
    }

    int[] storage = {0, 0}; // [have I checked yet, what was the result (interrupted or not)]
    int[] storage2 = {0, 0};

    @Override
    public Command routine() {
        return Commands.sequence(
            RobotContainer.selectPreset(Presets.BATTER),
            RobotContainer.shootSequence(),
            driveAndFetchFixed(grabFirstPath.get(note1), 1.0),
            scoreMaybeBail(scorePath.get(note1), grabSecondPath.get(note2), bailFromPath.get(note1), bailTime, storage)
            // scoreMaybeBail(scorePath.get(note2), grabSecondPath.get(note3), bailFromPath.get(note2), bailTime, storage2),

            // driveAndIntake(scorePath.get(note3), () -> true),
            // RobotContainer.shootSequence()

            // Commands.race(
            //     Commands.sequence(
            //         driveAndIntake(scorePath.get(note1), () -> true),
            //         RobotContainer.shootSequence()
            //     ),
            //     Commands.sequence(
            //         waitSeconds(bailTime),
            //         waitUntil(() -> {
            //             if (storage[0] == 0) { // have not checked yet
            //                 boolean missedPickup = true;// !RobotContainer.feeder.noteInExitSensor();
            //                 storage[1] = missedPickup ? 1 : 0;
            //                 return missedPickup;
            //             } else { // only let it check once
            //                 return false;
            //             }
            //         })
            //     )
            // ),

            // Commands.print("RESULT OF INTERRUPTION: " + storage[1]),

            // Commands.select(
            //     Map.of(0, driveAndIntake(grabSecondPath.get(note2)), 1, driveAndIntake(bailFromPath.get(note1))),
            //     () -> storage[1]
            // ) 

            // driveAndIntake(scorePath.get(note1), () -> true),
            // RobotContainer.shootSequence(),
            // driveAndIntake(grabSecondPath.get(note2)),
            // driveAndIntake(scorePath.get(note2), () -> true),
            // RobotContainer.shootSequence(),
            // drive(grabSecondPath.get(note3)).alongWith(RobotContainer.intake())// let intaking actually finish
           
            // RobotContainer.hailMary()
            // driveAndIntake(scorePath.get(note3), () -> true),
            // RobotContainer.shootSequence()
        );
    }

    @Override
    public Route getRoute() {
        return new Route()
            .addPathOptions(grabFirstPath)
            .addPathOptions(scorePath)
            .addPathOptions(grabSecondPath);
    }
    
}
