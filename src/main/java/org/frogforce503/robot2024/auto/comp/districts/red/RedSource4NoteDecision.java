package org.frogforce503.robot2024.auto.comp.districts.red;

import static edu.wpi.first.units.Units.Rotation;
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
import org.frogforce503.lib.trajectory.Route.Tree;
import org.frogforce503.lib.trajectory.Waypoint.Tag;
import org.frogforce503.lib.util.Logic;
import org.frogforce503.robot2024.RobotContainer;
import org.frogforce503.robot2024.auto.AutoUtil.Waypoints;
import org.frogforce503.robot2024.fields.FieldConfig;
import org.frogforce503.robot2024.planners.ShotPlanner;
import org.frogforce503.robot2024.planners.ShotPlanner.Presets;
import org.frogforce503.robot2024.subsystems.NoteDetector.SIDE;
import org.frogforce503.robot2024.subsystems.drive.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RedSource4NoteDecision extends AutoMode {

    // private HashMap<CENTERLINE_NOTES, PlannedPath> grabFirstPath = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> grabSecondPath = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> bailFromPath = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> scorePath = new HashMap<>();
    // private HashMap<CENTERLINE_NOTES, PlannedPath> hailMaryPath = new HashMap<>();

    private PlannedPath goOutAndShootFirst;

    private Tree grabFirstTree;

    private Pose2d shootingPose;
    private double firstHint;

    private double bailTime = 1.125; //s

    //11.41, 5.9
    Waypoint.Tag tag;

    public RedSource4NoteDecision() {

        var startingPose = setupPose(Waypoints.facingGoal(new Translation2d(FieldConfig.getInstance().RED_INITIATION_LINE + 0.5, FieldConfig.getInstance().NOTE_A.getY())));

        var shootingPosition = new Translation2d(12.48, 2.6);
        // shootingPose = Waypoints.facingGoal(shootingPosition)
        //     .transformBy(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(10)));
        shootingPose = new Pose2d(shootingPosition, Rotation2d.fromDegrees(-144));
        // 2.5 arm, 0
        firstHint = ShotPlanner.getInstance().getRawDistanceToGoal(shootingPosition);

        // goOutAndShootFirst = SwervePathBuilder.generate(4.5, 3.75, 0, 0.375,
        //     Waypoint.fromHolonomicPose(startingPose),
        //     Waypoint.fromDifferentialPose(shootingPose)
        // );

        var grabFirstTrunk = SwervePathBuilder.generate(5.5, 4.75, 0.0, 3.5,
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(new Translation2d(FieldConfig.getInstance().RED_WING_LINE + 2.25, FieldConfig.getInstance().NOTE_D.getY() + 0.5), null, new Rotation2d(Math.PI)),
            new Waypoint(new Translation2d(FieldConfig.getInstance().RED_WING_LINE + 0.5, FieldConfig.getInstance().NOTE_D.getY()), new Rotation2d(Math.PI), new Rotation2d(Math.PI))
        );

        var grabDBranch = SwervePathBuilder.generate(5.5, 4.75, 3.5, 0.0,
            Waypoint.fromHolonomicPose(grabFirstTrunk.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().NOTE_D.plus(new Translation2d(-Units.feetToMeters(0.0), Units.inchesToMeters(0))), new Rotation2d(Math.PI), new Rotation2d(Math.PI))
        );

        var grabEBranch = SwervePathBuilder.generate(5.5, 4.75, 3.5, 0,
            Waypoint.fromHolonomicPose(grabFirstTrunk.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().NOTE_E.plus(new Translation2d(-Units.feetToMeters(0.0), Units.inchesToMeters(0))), new Rotation2d(Math.PI), Rotation2d.fromDegrees(154))
        );

        grabFirstTree = new Tree()
            .withTrunk(grabFirstTrunk)
            .withBranches(grabDBranch, grabEBranch)
            .withNotes(CENTERLINE_NOTES.D, CENTERLINE_NOTES.E)
            .withFallbackPath(grabEBranch);

        tag = new Waypoint.Tag();

        var scoreD = SwervePathBuilder.generate(5.0, 4.75,
            Waypoint.fromHolonomicPose(grabDBranch.getFinalHolonomicPose()),
            Waypoint.fromHolonomicPose(shootingPose.transformBy(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(5))))
        );

        var scoreE = SwervePathBuilder.generate(5.0, 4.75,
            Waypoint.fromHolonomicPose(grabEBranch.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_LEFT.plus(new Translation2d(0, -1.0))),
            Waypoint.fromHolonomicPose(shootingPose.transformBy(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(3))))
        );

        var grabFSecond = SwervePathBuilder.generate(5.5, 4.25, 0.0, 0,
            Waypoint.fromHolonomicPose(shootingPose.transformBy(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(0)))),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_LEFT.interpolate(FieldConfig.getInstance().RED_STAGE_RIGHT, 0.5).interpolate(FieldConfig.getInstance().RED_CENTER_STAGE, 0.5)),
            new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(Units.feetToMeters(0.25), 0)), null, new Rotation2d(Math.PI))
        );

        var podiumShot = FieldConfig.getInstance().NOTE_A.plus(new Translation2d(Units.feetToMeters(0.5), Units.feetToMeters(0.25)));
        var podiumPose = Waypoints.facingGoal(podiumShot).transformBy(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(4)));

        var scoreF = SwervePathBuilder.generate(5.5, 4.75, 0.0, 0,
            Waypoint.fromHolonomicPose(grabFSecond.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_LEFT.interpolate(FieldConfig.getInstance().RED_STAGE_RIGHT, 0.5).interpolate(FieldConfig.getInstance().RED_CENTER_STAGE, 0.5)),
            Waypoint.fromHolonomicPose(shootingPose.transformBy(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(0)))),
            new Waypoint(FieldConfig.getInstance().NOTE_A.plus(new Translation2d(0.5, -1))),
            new Waypoint(FieldConfig.getInstance().NOTE_A.plus(new Translation2d(1.0, 0.25))),
            Waypoint.fromHolonomicPose(podiumPose)
        );

        scorePath.put(CENTERLINE_NOTES.D, scoreD);
        scorePath.put(CENTERLINE_NOTES.E, scoreE);
        scorePath.put(CENTERLINE_NOTES.NONE, scoreD);
        scorePath.put(CENTERLINE_NOTES.F, scoreF);

        var bailFromDToE = SwervePathBuilder.generate(4.0, 3.5,
            Waypoint.fromPlannedPathState(scoreD.sample(bailTime)),
            new Waypoint(grabEBranch.getFinalHolonomicPose().getTranslation(), new Rotation2d(Math.PI), Rotation2d.fromDegrees(135))
        );
        
        var bailFromEtoF = SwervePathBuilder.generate(4.0, 3.5,
            Waypoint.fromPlannedPathState(scoreE.sample(bailTime)),
            Waypoint.fromDifferentialPose(grabFSecond.getFinalHolonomicPose())
        );

        bailFromPath.put(CENTERLINE_NOTES.D, bailFromDToE);
        bailFromPath.put(CENTERLINE_NOTES.E, bailFromEtoF);

        grabSecondPath.put(CENTERLINE_NOTES.E, SwervePathBuilder.reversedOf(scoreE, 5.5, 4.5));
        grabSecondPath.put(CENTERLINE_NOTES.F, grabFSecond);
    }

    @Override
    public CENTERLINE_NOTES[] priorityList() {
        return new CENTERLINE_NOTES[] {CENTERLINE_NOTES.D, CENTERLINE_NOTES.E, CENTERLINE_NOTES.F};
    }

    int[] storage = {0, 0};
    int[] storage2 = {0, 0};

    @Override
    public Command routine() {
        return Commands.sequence(
            RobotContainer.selectPreset(Presets.BATTER),
            RobotContainer.shootSequence(),
            RobotContainer.hintAng(2.0, 1.0),

            drive(grabFirstTree.trunk),
            driveAndFetchFixed(grabFirstTree.branches[0], 1.0).withTimeout(grabFirstTree.branches[0].getTotalTimeSeconds()),
            // decide(grabFirstTree, this::perspective, tag),

            // Commands.either(
            scoreMaybeBail(scorePath.get(CENTERLINE_NOTES.D), grabSecondPath.get(CENTERLINE_NOTES.E), bailFromPath.get(CENTERLINE_NOTES.D), bailTime, storage).andThen(() -> { storage[0] = 0; storage[1] = 0; }),
            //     Commands.none(),
            //     () -> tag.getNote() == CENTERLINE_NOTES.D  
            // ),
            RobotContainer.hintAng(2.0, 3.0),
            scoreMaybeBail(scorePath.get(CENTERLINE_NOTES.E), grabSecondPath.get(CENTERLINE_NOTES.F), bailFromPath.get(CENTERLINE_NOTES.E), bailTime, storage),
            RobotContainer.hintAng(0, 10),
            Commands.parallel(
                drive(scorePath.get(CENTERLINE_NOTES.F)),
                RobotContainer.startAim(),
                Commands.sequence(
                    Commands.sequence(
                        RobotContainer.intake(),
                        Commands.run(() -> {})
                    ).withTimeout(scorePath.get(CENTERLINE_NOTES.F).getTotalTimeSeconds() - 0.5),
                    RobotContainer.shootContinuous()
                ),
                Commands.sequence(
                    Commands.waitSeconds(scorePath.get(CENTERLINE_NOTES.F).getTotalTimeSeconds() - 2),
                    Commands.waitUntil(Logic.not(RobotContainer.feeder::noteInExitSensor)),
                    RobotContainer.hintAng(0, 7)
                )
            )

            // scoreMaybeBail(scorePath.get(tag.getNote()), grabSecondPath.get(note3), bailFromPath.get(note2), bailTime, storage)

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

    private SIDE perspective(CENTERLINE_NOTES note) {
        return (note == CENTERLINE_NOTES.D) ? SIDE.CENTER : (note == CENTERLINE_NOTES.E ? SIDE.RIGHT : SIDE.RIGHT);
    }

    @Override
    public Route getRoute() {
        return new Route()
            .addTrees(grabFirstTree)
            .addPathOptions(scorePath)
            .addPathOptions(bailFromPath)
            .addPathOptions(grabSecondPath);
    }
    
}
