/*
 * Shoots preload at batter then picks up and shoots 3 ceterline notes
 */


package org.frogforce503.robot2024.auto.comp.WorkingAutons.blue;

import java.util.HashMap;

import org.frogforce503.lib.auto.AutoChooser.CENTERLINE_NOTES;
import org.frogforce503.lib.auto.AutoMode;
import org.frogforce503.lib.trajectory.PlannedPath;
import org.frogforce503.lib.trajectory.Route;
import org.frogforce503.lib.trajectory.SwervePathBuilder;
import org.frogforce503.lib.trajectory.Waypoint;
import org.frogforce503.robot2024.RobotContainer;
import org.frogforce503.robot2024.auto.AutoUtil.Waypoints;
import org.frogforce503.robot2024.fields.FieldConfig;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class BlueSource4NoteOFSZ_SHIFT extends AutoMode {

    private HashMap<CENTERLINE_NOTES, PlannedPath> grabFirstPath = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> grabSecondPath = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> scorePath = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, Pair<Double, Double>> aimMap = new HashMap<>();

    PlannedPath driveDtoE, driveDtoF, driveEtoD, driveEtoF, driveFtoD, driveFtoE;

    private HashMap<CENTERLINE_NOTES, PlannedPath> shiftFromDPath = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> shiftFromEPath = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> shiftFromFPath = new HashMap<>();

    private HashMap<CENTERLINE_NOTES, HashMap<CENTERLINE_NOTES, PlannedPath>> shiftPath = new HashMap<>();

    PlannedPath scoreFirstPathPart1, scoreFirstPathPart2, scoreSecondPathPart1, scoreSecondPathPart2;

    private double timeToWaitUntilBail = 1.0;

    private double maxVel = 4.25;
    private double maxAcc = 4.75;

    private double maxVel_SHIFT = 4.0;
    private double maxAcc_SHIFT = 3.5;

    private CENTERLINE_NOTES note1, note2, note3;

    public BlueSource4NoteOFSZ_SHIFT(CENTERLINE_NOTES note1, CENTERLINE_NOTES note2, CENTERLINE_NOTES note3) {

        aimMap.put(CENTERLINE_NOTES.D, Pair.of(0.0, 1.0));
        aimMap.put(CENTERLINE_NOTES.E, Pair.of(0.0, 1.0));
        aimMap.put(CENTERLINE_NOTES.F, Pair.of(0.0, 1.0));

        var startingPose = setupPose(
            Waypoints.facingGoal(new Translation2d(FieldConfig.getInstance().BLUE_INITIATION_LINE - 0.5, FieldConfig.getInstance().NOTE_I.getY()))
        );

        Pose2d shotPose = Waypoints.facingGoal(
            FieldConfig.getInstance().BLUE_STAGE_LEFT.plus(new Translation2d(-1.5, -0.5))
        );

        var grabDFirst = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_RIGHT.plus(new Translation2d(0.0, -1.0)), null, Rotation2d.fromDegrees(-20)),
            new Waypoint(FieldConfig.getInstance().NOTE_D, null, Rotation2d.fromDegrees(-20))
        );

        var grabEFirst = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_RIGHT.plus(new Translation2d(0, -1.0)), null, Rotation2d.fromDegrees(-10)),
            new Waypoint(FieldConfig.getInstance().NOTE_E, null, Rotation2d.fromDegrees(-10))
        );

        var grabFFirst = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().BLUE_CENTER_STAGE.plus(new Translation2d(0.0, -0.82)), null, new Rotation2d()),
            new Waypoint(FieldConfig.getInstance().NOTE_F, null, new Rotation2d())
        );

        grabFirstPath.put(CENTERLINE_NOTES.D, grabDFirst);
        grabFirstPath.put(CENTERLINE_NOTES.E, grabEFirst);
        grabFirstPath.put(CENTERLINE_NOTES.F, grabFFirst);

        var grabDSecond = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(shotPose).withHolonomicRotation(Rotation2d.fromDegrees(-20)),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_RIGHT.interpolate(FieldConfig.getInstance().BLUE_STAGE_LEFT, 0.5), null, Rotation2d.fromDegrees(-20)),
            new Waypoint(FieldConfig.getInstance().NOTE_D, null, Rotation2d.fromDegrees(-20))
        );

        var grabESecond = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(shotPose).withHolonomicRotation(Rotation2d.fromDegrees(-15)),
            new Waypoint(FieldConfig.getInstance().NOTE_E, null, Rotation2d.fromDegrees(-15))
        );

        var grabFSecond = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(shotPose).withHolonomicRotation(new Rotation2d()),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_RIGHT.interpolate(FieldConfig.getInstance().BLUE_STAGE_LEFT, 0.5), null, new Rotation2d()),
            new Waypoint(FieldConfig.getInstance().NOTE_F, null, new Rotation2d())
        );

        grabSecondPath.put(CENTERLINE_NOTES.D, grabDSecond);
        grabSecondPath.put(CENTERLINE_NOTES.E, grabESecond);
        grabSecondPath.put(CENTERLINE_NOTES.F, grabFSecond);

        var scoreD = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(grabDFirst.getFinalHolonomicPose()).withHolonomicRotation(shotPose.getRotation()),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_RIGHT.interpolate(FieldConfig.getInstance().BLUE_STAGE_LEFT, 0.5), null, shotPose.getRotation()),
            Waypoint.fromHolonomicPose(shotPose)
        );

        var scoreE = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(grabEFirst.getFinalHolonomicPose()).withHolonomicRotation(shotPose.getRotation()),
            Waypoint.fromHolonomicPose(shotPose)
        );
        
        var scoreF = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(grabFFirst.getFinalHolonomicPose()).withHolonomicRotation(shotPose.getRotation()),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_RIGHT.interpolate(FieldConfig.getInstance().BLUE_STAGE_LEFT, 0.5), null, shotPose.getRotation()),
            Waypoint.fromHolonomicPose(shotPose)
        );

        scorePath.put(CENTERLINE_NOTES.D, scoreD);
        scorePath.put(CENTERLINE_NOTES.E, scoreE);
        scorePath.put(CENTERLINE_NOTES.F, scoreF);

        this.note1 = note1;
        this.note2 = note2;
        this.note3 = note3;

        this.note1 = note1;
        this.note2 = note2;
        this.note3 = note3;

        driveDtoE = SwervePathBuilder.generate(maxVel_SHIFT, maxAcc_SHIFT,
            Waypoint.fromPlannedPathState(scoreD.sample(timeToWaitUntilBail)).withHolonomicRotation(new Rotation2d()),
            new Waypoint(grabEFirst.getFinalHolonomicPose().getTranslation(), null, new Rotation2d())
        );

        driveDtoF = SwervePathBuilder.generate(maxVel_SHIFT, maxAcc_SHIFT, 
            Waypoint.fromPlannedPathState(scoreD.sample(timeToWaitUntilBail)).withHolonomicRotation(Rotation2d.fromDegrees(25)),
            new Waypoint(grabFFirst.getFinalHolonomicPose().getTranslation(), null, Rotation2d.fromDegrees(25))
        );

        driveEtoD = SwervePathBuilder.generate(maxVel_SHIFT, maxAcc_SHIFT, 
            Waypoint.fromPlannedPathState(scoreE.sample(timeToWaitUntilBail)).withHolonomicRotation(Rotation2d.fromDegrees(-25)),
            new Waypoint(grabDFirst.getFinalHolonomicPose().getTranslation(), null, Rotation2d.fromDegrees(-25))
        );

        driveEtoF = SwervePathBuilder.generate(maxVel_SHIFT, maxAcc_SHIFT, 
            Waypoint.fromPlannedPathState(scoreE.sample(timeToWaitUntilBail)).withHolonomicRotation(new Rotation2d()),
            new Waypoint(grabFFirst.getFinalHolonomicPose().getTranslation(), null, new Rotation2d())
        );

        driveFtoD = SwervePathBuilder.generate(maxVel_SHIFT, maxAcc_SHIFT, 
            Waypoint.fromPlannedPathState(scoreF.sample(timeToWaitUntilBail)).withHolonomicRotation(Rotation2d.fromDegrees(-25)),
            new Waypoint(grabDFirst.getFinalHolonomicPose().getTranslation(), null, Rotation2d.fromDegrees(-25))
        );

        driveFtoE = SwervePathBuilder.generate(maxVel_SHIFT, maxAcc_SHIFT, 
            Waypoint.fromPlannedPathState(scoreF.sample(timeToWaitUntilBail)).withHolonomicRotation(Rotation2d.fromDegrees(-15)),
            new Waypoint(grabEFirst.getFinalHolonomicPose().getTranslation(), null, Rotation2d.fromDegrees(-15))
        );

        shiftFromDPath.put(CENTERLINE_NOTES.E, driveDtoE);
        shiftFromDPath.put(CENTERLINE_NOTES.F, driveDtoF);

        shiftFromEPath.put(CENTERLINE_NOTES.D, driveEtoD);
        shiftFromEPath.put(CENTERLINE_NOTES.F, driveEtoF);

        shiftFromFPath.put(CENTERLINE_NOTES.D, driveFtoD);
        shiftFromFPath.put(CENTERLINE_NOTES.E, driveFtoE);

        shiftPath.put(CENTERLINE_NOTES.D, shiftFromDPath);
        shiftPath.put(CENTERLINE_NOTES.E, shiftFromEPath);
        shiftPath.put(CENTERLINE_NOTES.F, shiftFromFPath);

        scoreFirstPathPart1 = SwervePathBuilder.generate(maxVel, maxAcc, 0.0, scorePath.get(note1).sample(timeToWaitUntilBail).velocityMetersPerSecond,
                                scorePath.get(note1).sampleTimeRange(0.0, timeToWaitUntilBail)
                            );

        scoreFirstPathPart2 = SwervePathBuilder.generate(maxVel, maxAcc, scorePath.get(note1).sample(timeToWaitUntilBail).velocityMetersPerSecond, 0.0,
                                scorePath.get(note1).sampleTimeRange(timeToWaitUntilBail, scorePath.get(note1).getTotalTimeSeconds())
                            );

        scoreSecondPathPart1 = SwervePathBuilder.generate(maxVel, maxAcc, 0.0, scorePath.get(note2).sample(timeToWaitUntilBail).velocityMetersPerSecond,
                                scorePath.get(note2).sampleTimeRange(0.0, timeToWaitUntilBail)
                            );
        
        scoreSecondPathPart2 = SwervePathBuilder.generate(maxVel, maxAcc, scorePath.get(note2).sample(timeToWaitUntilBail).velocityMetersPerSecond, 0.0,
                                scorePath.get(note2).sampleTimeRange(timeToWaitUntilBail, scorePath.get(note2).getTotalTimeSeconds())
                            );
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

            driveAndFetchFixed(grabFirstPath.get(this.note1), 0.75).deadlineWith(RobotContainer.hintAng(aimMap.get(this.note1))),

            driveAndIntake(scoreFirstPathPart1).alongWith(RobotContainer.hintAuton(aimMap.get(this.note1))),

            Commands.either(
                Commands.sequence(
                    driveAndIntake(scoreFirstPathPart2).alongWith(RobotContainer.hintAuton(aimMap.get(this.note1))).withTimeout(scorePath.get(this.note1).getTotalTimeSeconds() - timeToWaitUntilBail),
                    
                    RobotContainer.autonAdjust(),
                    RobotContainer.shootSequence().beforeStarting(RobotContainer.hintAuton(aimMap.get(this.note1), 0.05)),
                    
                    RobotContainer.hintAng(aimMap.get(this.note2)),
                    driveAndFetchFixed(grabSecondPath.get(this.note2), 0.75)
                ),
                Commands.sequence(
                    RobotContainer.hintAng(aimMap.get(this.note2)),
                    driveAndFetchFixed(shiftPath.get(this.note1).get(this.note2), 0.5)   
                ),
                RobotContainer.feeder::noteInEntrySensor
            ),

            driveAndIntake(scoreSecondPathPart1).alongWith(RobotContainer.hintAuton(aimMap.get(this.note2))),

            Commands.either(
                Commands.sequence(
                    driveAndIntake(scoreSecondPathPart2).alongWith(RobotContainer.hintAuton(aimMap.get(this.note2))).withTimeout(scorePath.get(this.note2).getTotalTimeSeconds() - timeToWaitUntilBail),

                    RobotContainer.autonAdjust(),
                    RobotContainer.shootSequence().beforeStarting(RobotContainer.hintAuton(aimMap.get(this.note2), 0.05)),
                
                    RobotContainer.hintAng(aimMap.get(this.note3)),
                    driveAndFetchFixed(grabSecondPath.get(this.note3), 0.75)
                ),
                Commands.sequence(
                    RobotContainer.hintAng(aimMap.get(this.note3)),
                    driveAndFetchFixed(shiftPath.get(this.note2).get(this.note3), 0.5)   
                ),
                RobotContainer.feeder::noteInEntrySensor
            ),

            driveAndIntake(scorePath.get(this.note3)).alongWith(RobotContainer.hintAuton(aimMap.get(this.note3))).withTimeout(scorePath.get(this.note3).getTotalTimeSeconds()),
            
            RobotContainer.autonAdjust(),
            RobotContainer.shootSequence().beforeStarting(RobotContainer.hintAuton(aimMap.get(this.note3), 0.05))
        );
    }

    @Override
    public Route getRoute() {
        return new Route(
            grabFirstPath.get(this.note1),
            scoreFirstPathPart1,
            scoreFirstPathPart2,
            grabSecondPath.get(this.note2),
            scoreSecondPathPart1,
            scoreSecondPathPart2,
            grabSecondPath.get(this.note3),
            scorePath.get(this.note3)
        );
    }
    
}
