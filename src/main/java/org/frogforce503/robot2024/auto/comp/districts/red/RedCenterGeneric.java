package org.frogforce503.robot2024.auto.comp.districts.red;

import static edu.wpi.first.wpilibj2.command.Commands.print;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import org.frogforce503.lib.auto.AutoMode;
import org.frogforce503.lib.auto.AutoChooser.CENTERLINE_NOTES;
import org.frogforce503.lib.trajectory.PlannedPath;
import org.frogforce503.lib.trajectory.Route;
import org.frogforce503.lib.trajectory.SwervePathBuilder;
import org.frogforce503.lib.trajectory.Waypoint;
import org.frogforce503.lib.trajectory.Route.Tree;
import org.frogforce503.lib.trajectory.Waypoint.Tag;
import org.frogforce503.robot2024.RobotContainer;
import org.frogforce503.robot2024.auto.AutoUtil.Waypoints;
import org.frogforce503.robot2024.fields.FieldConfig;
import org.frogforce503.robot2024.planners.ShotPlanner;
import org.frogforce503.robot2024.subsystems.NoteDetector.SIDE;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RedCenterGeneric extends AutoMode {

    private CENTERLINE_NOTES note1, note2, note3;
    private Pose2d shootingPosition;

    private PlannedPath grabB;
    private PlannedPath grabTrunk, grab1, grab2, grab3;
    private PlannedPath comeBackScore1, comeBackScore2, comeBackScore3, comeBackScoreRoot;
    private Tree grabTree, scoreTree;

    private double stableShotDistance;
    private Tag tag, tag2;

    private AutoCycleData data;

    Translation2d transit2;

    public RedCenterGeneric(CENTERLINE_NOTES note1, CENTERLINE_NOTES note2, CENTERLINE_NOTES note3) {
        this.note1 = note1;
        this.note2 = note2;
        this.note3 = note3;

        this.stableShotDistance = ShotPlanner.CENTRAL_SHOT_DISTANCE_RED - Units.feetToMeters(1.5);

        shootingPosition = Waypoints.facingGoal(
            new Translation2d(FieldConfig.getInstance().getFieldDimensions().getX() - stableShotDistance, FieldConfig.getInstance().NOTE_B.getY())
        );

        Pose2d startingPose = setupPose(Waypoints.facingGoal(new Translation2d(FieldConfig.getInstance().RED_INITIATION_LINE + 0.5, FieldConfig.getInstance().NOTE_B.getY())));

        grabB = SwervePathBuilder.generate(4.5,3.75, 
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().NOTE_B),
            Waypoint.fromHolonomicPose(shootingPosition)
        );


        // grab1 = goOutPath(note1);
        // grab2 = goOutPath(note2);
        // grab3 = goOutPath(note3);

        Translation2d transit1 = FieldConfig.getInstance().RED_CENTER_STAGE.interpolate(FieldConfig.getInstance().RED_STAGE_RIGHT, 0.25);
        Rotation2d transit1Direction = FieldConfig.getInstance().RED_STAGE_RIGHT.minus(FieldConfig.getInstance().RED_CENTER_STAGE).getAngle().plus(new Rotation2d(Math.PI/2));
        transit2 = FieldConfig.getInstance().RED_STAGE_LEFT.interpolate(FieldConfig.getInstance().RED_STAGE_RIGHT, 0.5);

        grabTrunk = SwervePathBuilder.generate(4.5, 3.75, 0, 3.0,
            Waypoint.fromHolonomicPose(shootingPosition),
            new Waypoint(transit1, transit1Direction, new Rotation2d(Math.PI)),
            new Waypoint(transit2, new Rotation2d(Math.PI), new Rotation2d(Math.PI))
        );

        grab1 = getNoteBranch(grabTrunk, 3.0, note1);
        grab2 = getNoteBranch(grabTrunk, 3.0, note2);
        grab3 = getNoteBranch(grabTrunk, 3.0, note3);

        grabTree = new Tree()
            .withTrunk(grabTrunk)
            .withBranches(grab1, grab2, grab3)
            .withNotes(note1, note2, note3);

        comeBackScore1 = SwervePathBuilder.reversedOf(grab1, 4.5, 3.75, 0, 3.0);
        comeBackScore2 = SwervePathBuilder.reversedOf(grab2, 4.5, 3.75, 0, 3.0);
        comeBackScore3 = SwervePathBuilder.reversedOf(grab3, 4.5, 3.75, 0, 3.0);
        
        comeBackScoreRoot = SwervePathBuilder.reversedOf(grabTrunk, 4.5, 3.75, 3.0, 0.0);

        scoreTree = new Tree()
            .withTrunk(comeBackScoreRoot)
            .withBranches(comeBackScore1, comeBackScore2, comeBackScore3)
            .withNotes(note1, note2, note3);

        // comeBackScore1 = getScoreBranch(grabTrunk, grab1);
        // comeBackScore2 = getScoreBranch(grabTrunk, grab2);
        // comeBackScore3 = getScoreBranch(grabTrunk, grab3);

        data = new AutoCycleData();
        tag = new Tag();
        tag2 = new Tag();
    }

    private PlannedPath getNoteBranch(PlannedPath trunk, double vi, CENTERLINE_NOTES note) {
        Translation2d notePos = note.position.get();
        Rotation2d pickupAngle = notePos.minus(transit2).getAngle();

        return SwervePathBuilder.generate(4.5, 3.75, vi, 0,
            Waypoint.fromHolonomicPose(trunk.getFinalHolonomicPose()),
            new Waypoint(notePos, pickupAngle, pickupAngle)
        );
    }

    private PlannedPath getScoreBranch(PlannedPath trunk, PlannedPath grabBranch) {
        List<Waypoint> first = SwervePathBuilder.reversedOf(grab1, 4.5, 3.75).getWaypoints();
        List<Waypoint> second = SwervePathBuilder.reversedOf(trunk, 4.5, 3.75).getWaypoints();
        first.addAll(second);
        return SwervePathBuilder.generate(4.5, 3.75, first);
    }

    // private PlannedPath 

    // private PlannedPath goOutPath(CENTERLINE_NOTES note) {
    //     if (note.transitThroughStage) {
    //         Translation2d transit1 = FieldConfig.getInstance().BLUE_CENTER_STAGE.interpolate(FieldConfig.getInstance().BLUE_STAGE_LEFT, 0.5);
    //         Rotation2d transit1Direction = FieldConfig.getInstance().BLUE_STAGE_LEFT.minus(FieldConfig.getInstance().BLUE_CENTER_STAGE).getAngle().minus(new Rotation2d(Math.PI/2));

    //         double interop = (note == CENTERLINE_NOTES.F) ? 0.5 : (note == CENTERLINE_NOTES.G ? 0.4 : 0.6);
    //         Translation2d transit2 = FieldConfig.getInstance().BLUE_STAGE_LEFT.interpolate(FieldConfig.getInstance().BLUE_STAGE_RIGHT, interop);

    //         Translation2d notePos = note.position.get();
    //         Rotation2d pickupAngle = notePos.minus(transit2).getAngle();

    //         return SwervePathBuilder.generate(4.5, 3.75,
    //             Waypoint.fromHolonomicPose(shootingPosition),
    //             new Waypoint(transit1, transit1Direction, new Rotation2d()),
    //             new Waypoint(transit2, pickupAngle.times(0.75), new Rotation2d()),
    //             new Waypoint(notePos, pickupAngle, pickupAngle)
    //         );
    //     } else { // only NOTE H
    //         Translation2d transit = FieldConfig.getInstance().BLUE_STAGE_LEFT.plus(new Translation2d(-0.25, 1));
     
    //         return SwervePathBuilder.generate(4.5, 3.75,
    //             Waypoint.fromHolonomicPose(shootingPosition),
    //             new Waypoint(transit, new Rotation2d(), new Rotation2d()),
    //             new Waypoint(note.position.get())
    //         );
    //     }
    // }

    @Override
    public Command routine() {
        return Commands.sequence(
            RobotContainer.shootSequence(),
            driveAndIntake(grabB).withTimeout(grabB.getTotalTimeSeconds() - 0.25), // shakiness at end
            RobotContainer.shootSequence(),
            decide(grabTree, this::perspective, tag),
            returningDecision(grabTree, scoreTree, this::perspective, tag)
            // RobotContainer.shootSequence()
            // decide(grabTree, this::perspective, tag2),
            // returningDecision(grabTree, scoreTree, this::perspective, tag2),
            // RobotContainer.shootSequence()

            // drive(grab1, 10.0)

            // drive(grabTrunk),
            // drive(grab1)
            // drive(comeBackScore1)
            
            // scoreCycle(grab1, comeBackScore1, SIDE.LEFT, data),
            // print("DONE WITH CYCLE 1" + data.toString()),
            // scoreCycle(grab2, comeBackScore2, SIDE.CENTER, data)
            
            
            // scoreCyle(grab1, comeBackScore1),
            // RobotContainer.shootSequence(),
            // scoreCyle(grab2, comeBackScore2),
            // RobotContainer.shootSequence(),
            // scoreCyle(grab3, comeBackScore3),
            // RobotContainer.shootSequence()
        );
    }

    private SIDE perspective(CENTERLINE_NOTES note) {
        return (note == CENTERLINE_NOTES.G) ? SIDE.RIGHT : (note == CENTERLINE_NOTES.F ? SIDE.CENTER : SIDE.LEFT);
    }

    @Override
    public CENTERLINE_NOTES[] priorityList() {
        return new CENTERLINE_NOTES[] {note1, note2, note3};
    }

    @Override
    public Route getRoute() {
        return new Route(grabB, grabTrunk, grab1, grab2, grab3);
    }
    
}
