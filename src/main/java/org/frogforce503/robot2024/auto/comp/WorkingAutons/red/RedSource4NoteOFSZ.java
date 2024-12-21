package org.frogforce503.robot2024.auto.comp.WorkingAutons.red;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import java.util.HashMap;

import org.frogforce503.lib.auto.AutoMode;
import org.frogforce503.lib.auto.AutoChooser.CENTERLINE_NOTES;
import org.frogforce503.lib.trajectory.PlannedPath;
import org.frogforce503.lib.trajectory.Route;
import org.frogforce503.lib.trajectory.SwervePathBuilder;
import org.frogforce503.lib.trajectory.Waypoint;
import org.frogforce503.robot2024.Robot;
import org.frogforce503.robot2024.RobotContainer;
import org.frogforce503.robot2024.auto.AutoUtil.Waypoints;
import org.frogforce503.robot2024.fields.FieldConfig;
import org.frogforce503.robot2024.subsystems.NoteDetector.SIDE;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** fixed bail paths */

public class RedSource4NoteOFSZ extends AutoMode {

    private HashMap<CENTERLINE_NOTES, PlannedPath> grabFirstPath = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> grabSecondPath = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> scorePath = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, Pair<Double, Double>> aimMap = new HashMap<>();

    private CENTERLINE_NOTES note1, note2, note3;

    private Pose2d shootingPose;

    public RedSource4NoteOFSZ(CENTERLINE_NOTES note1, CENTERLINE_NOTES note2, CENTERLINE_NOTES note3) {

        var startingPose = setupPose(Waypoints.facingGoal(new Translation2d(FieldConfig.getInstance().RED_INITIATION_LINE + 0.5, FieldConfig.getInstance().NOTE_A.getY())));

        var shootingPositionX = FieldConfig.getInstance().RED_CENTER_STAGE.interpolate(FieldConfig.getInstance().RED_STAGE_LEFT, 0.5).getX() + 1;
        var shootingPosition = new Translation2d(shootingPositionX, FieldConfig.getInstance().RED_SPEAKER.getY());

        shootingPose = new Pose2d(shootingPosition, new Rotation2d(Math.PI));

        aimMap.put(CENTERLINE_NOTES.D, Pair.of(0.0, 1.5));
        aimMap.put(CENTERLINE_NOTES.E, Pair.of(0.0, 1.5));
        aimMap.put(CENTERLINE_NOTES.F, Pair.of(0.0, 1.5));

        Rotation2d noteDToStart = FieldConfig.getInstance().NOTE_D.minus(startingPose.getTranslation()).getAngle();
        
        var grabDFirst = SwervePathBuilder.generate(4.6, 4.75, // 5.0, 4.75
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_LEFT.minus(new Translation2d(0, 1.5)), null, noteDToStart),
            new Waypoint(FieldConfig.getInstance().NOTE_D, noteDToStart, noteDToStart)
        );

        var grabEFirst = SwervePathBuilder.generate(4.6, 4.75,
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_LEFT.minus(new Translation2d(0, 1.0)), null, new Rotation2d(Math.PI).plus(Rotation2d.fromDegrees(5))),
            new Waypoint(FieldConfig.getInstance().NOTE_E, FieldConfig.getInstance().NOTE_E.minus(startingPose.getTranslation()).getAngle(), FieldConfig.getInstance().NOTE_E.minus(startingPose.getTranslation()).getAngle().plus(Rotation2d.fromDegrees(20)))
        );

        var grabFFirst = SwervePathBuilder.generate(4.6, 4.75,
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().RED_CENTER_STAGE.plus(new Translation2d(0.0, -0.82)), null, shootingPose.getRotation()),
            new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(Units.feetToMeters(0.0), Units.inchesToMeters(0))), null, new Rotation2d(Math.PI))
        );

        grabFirstPath.put(CENTERLINE_NOTES.D, grabDFirst);
        grabFirstPath.put(CENTERLINE_NOTES.E, grabEFirst);
        grabFirstPath.put(CENTERLINE_NOTES.F, grabFFirst);

        var scoreD = SwervePathBuilder.generate(4.6, 4.75,
            Waypoint.fromHolonomicPose(grabDFirst.getFinalHolonomicPose()).withHolonomicRotation(Rotation2d.fromDegrees(210)),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_LEFT.plus(new Translation2d(-0.5, 1.25)), null, Rotation2d.fromDegrees(200)),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(1, -0.5)), null, new Rotation2d(Math.PI)),
            Waypoint.fromHolonomicPose(shootingPose)
        );

        var scoreE = SwervePathBuilder.generate(4.6, 4.75,
            Waypoint.fromHolonomicPose(grabEFirst.getFinalHolonomicPose()).withHolonomicRotation(Rotation2d.fromDegrees(210)),
            Waypoint.fromHolonomicPose(shootingPose.transformBy(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(-3))))
        );
        
        var scoreF = SwervePathBuilder.generate(4.6, 4.75,
            Waypoint.fromHolonomicPose(grabFFirst.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_LEFT.plus(new Translation2d(0.0, 1.4)), null, new Rotation2d(Math.PI)),
            Waypoint.fromHolonomicPose(shootingPose)
        );

        scorePath.put(CENTERLINE_NOTES.D, scoreD);
        scorePath.put(CENTERLINE_NOTES.E, scoreE);
        scorePath.put(CENTERLINE_NOTES.F, scoreF);

        grabSecondPath.put(CENTERLINE_NOTES.D, SwervePathBuilder.reversedOf(scoreD, 5.0, 4.75));
        grabSecondPath.put(CENTERLINE_NOTES.E, SwervePathBuilder.reversedOf(scoreE, 5.0, 4.75));
        grabSecondPath.put(CENTERLINE_NOTES.F, SwervePathBuilder.reversedOf(scoreF, 5.0, 4.75));

        this.note1 = note1;
        this.note2 = note2;
        this.note3 = note3;
    }

    @Override
    public CENTERLINE_NOTES[] priorityList() {
        return new CENTERLINE_NOTES[] {note1, note2, note3};
    }

    @Override
    public Command routine() {
        return Commands.sequence(
            RobotContainer.hintAng(0.0, 15.0),
            RobotContainer.shootSequence(),

            RobotContainer.hintAng(aimMap.get(this.note1)),
            driveAndFetchFixed(grabFirstPath.get(note1), 0.7),

            driveAndIntake(scorePath.get(note1)).alongWith(RobotContainer.shooter.rampUp(), RobotContainer.hintAng(aimMap.get(this.note1))).withTimeout(scorePath.get(note1).getTotalTimeSeconds() - 0.3),
                    
            RobotContainer.shootSequence().beforeStarting(Commands.parallel(RobotContainer.shooter.rampUp(), RobotContainer.hintAng(aimMap.get(this.note1))).withTimeout(0.1)),
            
            RobotContainer.hintAng(aimMap.get(this.note2)),
            driveAndFetchFixed(grabSecondPath.get(note2), 0.7),

            driveAndIntake(scorePath.get(note2)).alongWith(RobotContainer.shooter.rampUp(), RobotContainer.hintAng(aimMap.get(this.note2))).withTimeout(scorePath.get(note2).getTotalTimeSeconds() - 0.1),
            RobotContainer.shootSequence().beforeStarting(Commands.parallel(RobotContainer.shooter.rampUp(), RobotContainer.hintAng(aimMap.get(this.note2))).withTimeout(0.1)),
        
            RobotContainer.hintAng(aimMap.get(this.note3)),
            driveAndFetchFixed(grabSecondPath.get(note3), 0.7),

            driveAndIntake(scorePath.get(note3)).alongWith(RobotContainer.shooter.rampUp(), RobotContainer.hintAng(aimMap.get(this.note3))).withTimeout(scorePath.get(note3).getTotalTimeSeconds() - 0.1),
            RobotContainer.shootSequence().beforeStarting(Commands.parallel(RobotContainer.shooter.rampUp(), RobotContainer.hintAng(aimMap.get(this.note3))).withTimeout(0.1))
        );
    }

    @Override
    public Route getRoute() {
        return new Route()
            .addPathOptions(grabFirstPath)
            .addPathOptions(grabSecondPath);
    }
    
}
