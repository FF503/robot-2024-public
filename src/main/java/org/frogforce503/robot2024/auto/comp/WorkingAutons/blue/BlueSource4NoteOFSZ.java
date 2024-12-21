/*
 * Shoots preload at batter then picks up and shoots 3 ceterline notes
 */


package org.frogforce503.robot2024.auto.comp.WorkingAutons.blue;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

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
import org.frogforce503.robot2024.subsystems.NoteDetector.SIDE;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class BlueSource4NoteOFSZ extends AutoMode {

    private HashMap<CENTERLINE_NOTES, PlannedPath> grabFirstPath = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> grabSecondPath = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> bailFromPath = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> scorePath = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, Pair<Double, Double>> aimMap = new HashMap<>();


    private CENTERLINE_NOTES note1, note2, note3;

    private Pose2d shootingPose;

    private double bailTime = 0.475; //s

    //11.41, 5.9
    Waypoint.Tag tag;

    public BlueSource4NoteOFSZ(CENTERLINE_NOTES note1, CENTERLINE_NOTES note2, CENTERLINE_NOTES note3) {

        var startingPose = setupPose(Waypoints.facingGoal(new Translation2d(FieldConfig.getInstance().BLUE_INITIATION_LINE - 0.5, FieldConfig.getInstance().NOTE_I.getY())));

        var shootingPositionX = FieldConfig.getInstance().BLUE_CENTER_STAGE.interpolate(FieldConfig.getInstance().BLUE_STAGE_RIGHT, 0.5).getX() - 0.5;
        var shootingPosition = new Translation2d(shootingPositionX - Units.feetToMeters(1), FieldConfig.getInstance().BLUE_SPEAKER.getY() - Units.feetToMeters(1.5));

        shootingPose = Waypoints.facingGoal(shootingPosition);
        // 2.5 arm, 0

        // Arm might be around 0.875-1 for working PID wrist
        aimMap.put(CENTERLINE_NOTES.D, Pair.of(0.0, 2.0)); // prev arm = 2.0
        aimMap.put(CENTERLINE_NOTES.E, Pair.of(0.0, 2.0));
        aimMap.put(CENTERLINE_NOTES.F, Pair.of(0.0, 2.0)); // prev arm = 2.0


        // var grabDFirst = SwervePathBuilder.generate(1, 1, // 5.0, 4.75
        //     Waypoint.fromHolonomicPose(startingPose),
        //     new Waypoint(new Translation2d(FieldConfig.getInstance().BLUE_WING_LINE - 2.25, FieldConfig.getInstance().NOTE_D.getY() + 0.5), null, new Rotation2d()),
        //     new Waypoint(new Translation2d(FieldConfig.getInstance().BLUE_WING_LINE - 0.5, FieldConfig.getInstance().NOTE_D.getY()), new Rotation2d(), new Rotation2d()),
        //     new Waypoint(FieldConfig.getInstance().NOTE_D.plus(new Translation2d(Units.feetToMeters(0.0), Units.inchesToMeters(0))), new Rotation2d(), new Rotation2d())
        // );


        Rotation2d noteDToStart = FieldConfig.getInstance().NOTE_D.minus(startingPose.getTranslation()).getAngle();
        var grabDFirst = SwervePathBuilder.generate(5.0, 4.75, // 5.0, 4.75
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_RIGHT.minus(new Translation2d(0, 1.25)), null, noteDToStart),
            // new Waypoint(new Translation2d(5, noteDToStart), noteDToStart, noteDToStart),
            // new Waypoint(new Translation2d(10, noteDToStart), noteDToStart, noteDToStart),
            new Waypoint(FieldConfig.getInstance().NOTE_D.plus(new Translation2d(Units.feetToMeters(0.0), Units.inchesToMeters(0))), noteDToStart, noteDToStart)
        );

        var grabEFirst = SwervePathBuilder.generate(4.8, 4.75,
            // new ArrayList<>() {{
            //     addAll(grabDFirst.getWaypoints().subList(0, 1)); // 0, 3
            //     add(new Waypoint(FieldConfig.getInstance().NOTE_E.plus(new Translation2d(Units.feetToMeters(0.0), Units.inchesToMeters(0))), new Rotation2d(), Rotation2d.fromDegrees(26)));
            // }}
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_RIGHT.minus(new Translation2d(0, 1.1)), null, Rotation2d.fromDegrees(6)),
            new Waypoint(FieldConfig.getInstance().NOTE_E.plus(new Translation2d(Units.feetToMeters(0.0), Units.inchesToMeters(0))), new Rotation2d(), Rotation2d.fromDegrees(6))
        );

        var grabFFirst = SwervePathBuilder.generate(4.8, 4.75,
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().BLUE_CENTER_STAGE.interpolate(FieldConfig.getInstance().BLUE_STAGE_RIGHT, 0.5), Rotation2d.fromDegrees(25), null),
            new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(Units.feetToMeters(0.0), Units.inchesToMeters(0))), new Rotation2d(), new Rotation2d())
        );

        grabFirstPath.put(CENTERLINE_NOTES.D, grabDFirst);
        grabFirstPath.put(CENTERLINE_NOTES.E, grabEFirst);
        grabFirstPath.put(CENTERLINE_NOTES.F, grabFFirst);

        tag = new Waypoint.Tag();

        // var scoreD = SwervePathBuilder.generate(5.0, 4.75,
        //     Waypoint.fromHolonomicPose(grabDFirst.getFinalHolonomicPose()).withHolonomicRotation(Rotation2d.fromDegrees(-30)),
        //     new Waypoint(FieldConfig.getInstance().BLUE_STAGE_RIGHT.plus(new Translation2d(1.2, 0))),
        //     new Waypoint(FieldConfig.getInstance().BLUE_STAGE_LEFT.interpolate(FieldConfig.getInstance().BLUE_STAGE_RIGHT, 0.5), new Rotation2d(Math.PI), null),
        //     Waypoint.fromHolonomicPose(shootingPose.transformBy(new Transform2d(new Translation2d(0, 0), Rotation2d.fromDegrees(-6))))
        // );


        var scoreD = SwervePathBuilder.generate(4.8, 4.75,
            Waypoint.fromHolonomicPose(grabDFirst.getFinalHolonomicPose()).withHolonomicRotation(Rotation2d.fromDegrees(-30)),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_RIGHT.plus(new Translation2d(1.2, 0.25)), null, shootingPose.getRotation()),
            // new Waypoint(FieldConfig.getInstance().BLUE_STAGE_RIGHT.plus(new Translation2d(0.75, 0.875)), null, shootingPose.getRotation()),
            // new Waypoint(FieldConfig.getInstance().BLUE_STAGE_LEFT.interpolate(FieldConfig.getInstance().BLUE_STAGE_RIGHT, 0.6), new Rotation2d(Math.PI), null),
            Waypoint.fromHolonomicPose(shootingPose/*.transformBy(new Transform2d(new Translation2d(0, 0), Rotation2d.fromDegrees(-6)))*/)
        );


        var scoreE = SwervePathBuilder.generate(4.8, 4.75,
            Waypoint.fromHolonomicPose(grabEFirst.getFinalHolonomicPose()).withHolonomicRotation(Rotation2d.fromDegrees(-30)), //-30
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_LEFT.interpolate(FieldConfig.getInstance().BLUE_STAGE_RIGHT, 0.5), null, shootingPose.getRotation()),
            Waypoint.fromHolonomicPose(shootingPose)
        );
        
        var scoreF = SwervePathBuilder.generate(4.8, 4.75, 0.0, 0,
            Waypoint.fromHolonomicPose(grabFFirst.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_LEFT.interpolate(FieldConfig.getInstance().BLUE_STAGE_RIGHT, 0.5), new Rotation2d(Math.PI), null),
            Waypoint.fromHolonomicPose(shootingPose)
            
        );

        scorePath.put(CENTERLINE_NOTES.D, scoreD);
        scorePath.put(CENTERLINE_NOTES.E, scoreE);
        scorePath.put(CENTERLINE_NOTES.F, scoreF);

        var grabESecond = SwervePathBuilder.generate(4.8, 4.75,
            Waypoint.fromHolonomicPose(shootingPose),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_LEFT.interpolate(FieldConfig.getInstance().BLUE_STAGE_RIGHT, 0.5), null, Rotation2d.fromDegrees(-30)),
            Waypoint.fromHolonomicPose(grabEFirst.getFinalHolonomicPose()).withHolonomicRotation(Rotation2d.fromDegrees(-30))
        );

        grabSecondPath.put(CENTERLINE_NOTES.D, SwervePathBuilder.reversedOf(scoreD, 5.0, 4.75));
        grabSecondPath.put(CENTERLINE_NOTES.E, grabESecond);
        grabSecondPath.put(CENTERLINE_NOTES.F, SwervePathBuilder.reversedOf(scoreF, 5.0, 4.75));

        var bailFromDToE = SwervePathBuilder.generate(4.0, 3.5,
            Waypoint.fromPlannedPathState(scoreD.sample(bailTime)),
            new Waypoint(grabEFirst.getFinalHolonomicPose().getTranslation(), new Rotation2d(), Rotation2d.fromDegrees(45))
        );
        
        var bailFromEtoF = SwervePathBuilder.generate(4.0, 3.5,
            Waypoint.fromPlannedPathState(scoreE.sample(bailTime)),
            Waypoint.fromDifferentialPose(grabFFirst.getFinalHolonomicPose())
        );

        bailFromPath.put(CENTERLINE_NOTES.D, bailFromDToE);
        bailFromPath.put(CENTERLINE_NOTES.E, bailFromEtoF);

        this.note1 = note1;
        this.note2 = note2;
        this.note3 = note3;
    }

    @Override
    public CENTERLINE_NOTES[] priorityList() {
        return new CENTERLINE_NOTES[] {note1, note2, note3};
    }

    int[] storage = {0, 0};
    int[] storage2 = {0, 0};

    @Override
    public Command routine() {
        return Commands.sequence(
            RobotContainer.hintAng(0.0, 12.5),
            RobotContainer.shootSequence(), // TODO Change time out

            RobotContainer.hintAng(aimMap.get(this.note1)), // 0, 0
            driveAndFetchFixed(grabFirstPath.get(note1), 0.75).withTimeout(grabFirstPath.get(note1).getTotalTimeSeconds() + 0.5),
            driveAndIntake(scorePath.get(note1)),
            RobotContainer.shootSequence().beforeStarting(waitSeconds(0.125)),
            
            // scoreMaybeBail(scorePath.get(note1), grabSecondPath.get(note2), bailFromPath.get(note1), bailTime, storage).andThen(() -> { storage[0] = 0; storage[1] = 0; }),

            RobotContainer.hintAng(aimMap.get(this.note2)), // 0, 0
            driveAndFetchFixed(grabSecondPath.get(note2), 0.6).withTimeout(grabSecondPath.get(note2).getTotalTimeSeconds() + 0.5),
            driveAndIntake(scorePath.get(note2)),
            RobotContainer.shootSequence().beforeStarting(waitSeconds(0.125)),
            // // scoreMaybeBail(scorePath.get(note2), grabSecondPath.get(note3), bailFromPath.get(note2), bailTime, storage),
           
            RobotContainer.hintAng(aimMap.get(this.note3)), // 0, 0
            driveAndFetchFixed(grabSecondPath.get(note3), 1.0).withTimeout(grabSecondPath.get(note3).getTotalTimeSeconds()),
            driveAndIntake(scorePath.get(note3)),
            RobotContainer.shootSequence().beforeStarting(waitSeconds(0.125))
        );
    }

    private SIDE perspective(CENTERLINE_NOTES note) {
        return (note == CENTERLINE_NOTES.D) ? SIDE.CENTER : (note == CENTERLINE_NOTES.E ? SIDE.RIGHT : SIDE.RIGHT);
    }

    @Override
    public Route getRoute() {
        return new Route()
            .addPathOptions(grabFirstPath)
            // .addPathOptions(scorePath)
            // .addPathOptions(bailFromPath)
            .addPathOptions(grabSecondPath);
    }
    
}
