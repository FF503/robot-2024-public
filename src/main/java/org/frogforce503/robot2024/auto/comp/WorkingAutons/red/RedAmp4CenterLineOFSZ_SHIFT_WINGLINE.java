package org.frogforce503.robot2024.auto.comp.WorkingAutons.red;

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

public class RedAmp4CenterLineOFSZ_SHIFT_WINGLINE extends AutoMode {

    private HashMap<CENTERLINE_NOTES, PlannedPath> grabFirstPath = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> grabSecondPath = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> scorePath = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, Pair<Double, Double>> aimMap = new HashMap<>();

    PlannedPath driveFtoG, driveFtoH, driveGtoF, driveGtoH, driveHtoF, driveHtoG;

    private HashMap<CENTERLINE_NOTES, PlannedPath> shiftFromFPath = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> shiftFromGPath = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> shiftFromHPath = new HashMap<>();

    private HashMap<CENTERLINE_NOTES, HashMap<CENTERLINE_NOTES, PlannedPath>> shiftPath = new HashMap<>();

    PlannedPath grabFirstPathPart1, grabFirstPathPart2, scoreFirstPathPart1, scoreFirstPathPart2, scoreSecondPathPart1, scoreSecondPathPart2;

    private double shootPreloadTime = 1.0; // Can tune so that shot goes in
    private double timeToWaitUntilBail = 1.0; // Test smaller times

    private double maxVel = 4.25;
    private double maxAcc = 4.75;

    private double maxVel_PRELOAD = 1.0; // Can tune so that shot goes in
    private double maxAcc_PRELOAD = 3.0; // Can tune so that shot goes in

    private double maxVel_SHOT = 4.0; // Can tune so that shot goes in
    private double maxAcc_SHOT = 3.0; // Can tune so that shot goes in

    private double maxVel_SHIFT = 4.0;
    private double maxAcc_SHIFT = 3.5;

    private CENTERLINE_NOTES note1, note2, note3;

    public RedAmp4CenterLineOFSZ_SHIFT_WINGLINE(CENTERLINE_NOTES note1, CENTERLINE_NOTES note2, CENTERLINE_NOTES note3) {

        aimMap.put(CENTERLINE_NOTES.F, Pair.of(1.0, 0.0));
        aimMap.put(CENTERLINE_NOTES.G, Pair.of(1.0, 0.0));
        aimMap.put(CENTERLINE_NOTES.H, Pair.of(1.0, 0.0));

        Pose2d startingPose = setupPose(
            Waypoints.facingGoal(
                FieldConfig.getInstance().NOTE_C.interpolate(FieldConfig.getInstance().NOTE_B, 0.5).plus(new Translation2d(1.0, 0.0))
            )
        );

        Pose2d shotPose = Waypoints.facingGoal(
            FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0.5, 1.0))
        );

        var grabHFirst = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().NOTE_H, null, new Rotation2d(Math.PI))
        );

        var grabGFirst = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0.0, 1.0))),
            new Waypoint(FieldConfig.getInstance().NOTE_G, null, new Rotation2d(Math.PI))
        );

        var grabFFirst = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(-0.5, 1.0)), null, new Rotation2d(Math.PI).plus(Rotation2d.fromDegrees(30))),
            new Waypoint(FieldConfig.getInstance().NOTE_F, null, new Rotation2d(Math.PI).plus(Rotation2d.fromDegrees(30)))
        );

        grabFirstPath.put(CENTERLINE_NOTES.H, grabHFirst);
        grabFirstPath.put(CENTERLINE_NOTES.G, grabGFirst);
        grabFirstPath.put(CENTERLINE_NOTES.F, grabFFirst);

        var grabHSecond = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(shotPose).withHolonomicRotation(new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().NOTE_H, null, new Rotation2d(Math.PI))
        );

        var grabGSecond = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(shotPose).withHolonomicRotation(new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(-1.0, 1.0)), null, new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().NOTE_G, null, new Rotation2d(Math.PI))
        );

        var grabFSecond = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(shotPose).withHolonomicRotation(new Rotation2d(Math.PI).plus(Rotation2d.fromDegrees(30))),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(-1.0, 1.0)), null, new Rotation2d(Math.PI).plus(Rotation2d.fromDegrees(30))),
            new Waypoint(FieldConfig.getInstance().NOTE_F, null, new Rotation2d(Math.PI).plus(Rotation2d.fromDegrees(30)))
        );

        grabSecondPath.put(CENTERLINE_NOTES.H, grabHSecond);
        grabSecondPath.put(CENTERLINE_NOTES.G, grabGSecond);
        grabSecondPath.put(CENTERLINE_NOTES.F, grabFSecond);

        var scoreH = SwervePathBuilder.generate(maxVel_SHOT, maxAcc_SHOT,
            Waypoint.fromHolonomicPose(grabHFirst.getFinalHolonomicPose()).withHolonomicRotation(shotPose.getRotation()),
            Waypoint.fromHolonomicPose(shotPose)
        );

        var scoreG = SwervePathBuilder.generate(maxVel_SHOT, maxAcc_SHOT,
            Waypoint.fromHolonomicPose(grabGFirst.getFinalHolonomicPose()).withHolonomicRotation(shotPose.getRotation()),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(-1.0, 1.0)), null, shotPose.getRotation()),
            Waypoint.fromHolonomicPose(shotPose)
        );

        var scoreF = SwervePathBuilder.generate(maxVel_SHOT, maxAcc_SHOT,
            Waypoint.fromHolonomicPose(grabFSecond.getFinalHolonomicPose()).withHolonomicRotation(shotPose.getRotation()),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(-1.0, 1.0)), null, shotPose.getRotation()),
            Waypoint.fromHolonomicPose(shotPose)
        );

        scorePath.put(CENTERLINE_NOTES.H, scoreH);
        scorePath.put(CENTERLINE_NOTES.G, scoreG);
        scorePath.put(CENTERLINE_NOTES.F, scoreF);

        this.note1 = note1;
        this.note2 = note2;
        this.note3 = note3;

        driveFtoG = SwervePathBuilder.generate(maxVel_SHIFT, maxAcc_SHIFT, 
            Waypoint.fromPlannedPathState(scoreF.sample(timeToWaitUntilBail)).withHolonomicRotation(new Rotation2d(Math.PI)),
            new Waypoint(grabGFirst.getFinalHolonomicPose().getTranslation(), null, new Rotation2d(Math.PI))
        );

        driveFtoH = SwervePathBuilder.generate(maxVel_SHIFT, maxAcc_SHIFT, 
            Waypoint.fromPlannedPathState(scoreF.sample(timeToWaitUntilBail)).withHolonomicRotation(new Rotation2d(Math.PI).plus(Rotation2d.fromDegrees(-30))),
            new Waypoint(FieldConfig.getInstance().NOTE_G.plus(new Translation2d(1.25, 0.5)), null, new Rotation2d(Math.PI).plus(Rotation2d.fromDegrees(-30))),
            new Waypoint(grabHFirst.getFinalHolonomicPose().getTranslation(), null, new Rotation2d(Math.PI).plus(Rotation2d.fromDegrees(-30)))
        );

        driveGtoF = SwervePathBuilder.generate(maxVel_SHIFT, maxAcc_SHIFT, 
            Waypoint.fromPlannedPathState(scoreG.sample(timeToWaitUntilBail)).withHolonomicRotation(new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(1.25, 0.5)), null, new Rotation2d(Math.PI)),
            new Waypoint(grabFFirst.getFinalHolonomicPose().getTranslation(), null, new Rotation2d(Math.PI))
        );

        driveGtoH = SwervePathBuilder.generate(maxVel_SHIFT, maxAcc_SHIFT, 
            Waypoint.fromPlannedPathState(scoreG.sample(timeToWaitUntilBail)).withHolonomicRotation(new Rotation2d(Math.PI).plus(Rotation2d.fromDegrees(-10))),
            new Waypoint(grabHFirst.getFinalHolonomicPose().getTranslation(), null, new Rotation2d(Math.PI).plus(Rotation2d.fromDegrees(-10)))
        );

        driveHtoF = SwervePathBuilder.generate(maxVel_SHIFT, maxAcc_SHIFT, 
            Waypoint.fromPlannedPathState(scoreH.sample(timeToWaitUntilBail)).withHolonomicRotation(new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(1.25, 0.5)), null, new Rotation2d(Math.PI)),
            new Waypoint(grabFFirst.getFinalHolonomicPose().getTranslation(), null, new Rotation2d(Math.PI))
        );

        driveHtoG = SwervePathBuilder.generate(maxVel_SHIFT, maxAcc_SHIFT, 
            Waypoint.fromPlannedPathState(scoreH.sample(timeToWaitUntilBail)).withHolonomicRotation(new Rotation2d(Math.PI).plus(Rotation2d.fromDegrees(20))),
            new Waypoint(grabGFirst.getFinalHolonomicPose().getTranslation(), null, new Rotation2d(Math.PI).plus(Rotation2d.fromDegrees(20)))
        );

        shiftFromFPath.put(CENTERLINE_NOTES.G, driveFtoG);
        shiftFromFPath.put(CENTERLINE_NOTES.H, driveFtoH);

        shiftFromGPath.put(CENTERLINE_NOTES.F, driveGtoF);
        shiftFromGPath.put(CENTERLINE_NOTES.H, driveGtoH);

        shiftFromHPath.put(CENTERLINE_NOTES.F, driveHtoF);
        shiftFromHPath.put(CENTERLINE_NOTES.G, driveHtoG);

        shiftPath.put(CENTERLINE_NOTES.F, shiftFromFPath);
        shiftPath.put(CENTERLINE_NOTES.G, shiftFromGPath);
        shiftPath.put(CENTERLINE_NOTES.H, shiftFromHPath);

        double velAfterPreloadShot = Math.min(maxVel_PRELOAD, grabFirstPath.get(note1).sample(shootPreloadTime).velocityMetersPerSecond);

        grabFirstPathPart1 = SwervePathBuilder.generate(maxVel_PRELOAD, maxAcc_PRELOAD, 0.0, velAfterPreloadShot,
                                Waypoint.fromHolonomicPose(grabFirstPath.get(note1).getInitialHolonomicPose()),
                                Waypoint.fromHolonomicPose(Waypoints.facingGoal(grabFirstPath.get(note1).sample(shootPreloadTime).poseMeters)) // Makes sure robot always faces goal when shooting
                            );

        grabFirstPathPart2 = SwervePathBuilder.generate(maxVel, maxAcc, velAfterPreloadShot, 0.0,
                                grabFirstPath.get(note1).sampleTimeRange(shootPreloadTime, grabFirstPath.get(note1).getTotalTimeSeconds())
                            );

        scoreFirstPathPart1 = SwervePathBuilder.generate(maxVel_SHOT, maxAcc_SHOT, 0.0, scorePath.get(note1).sample(timeToWaitUntilBail).velocityMetersPerSecond,
                                scorePath.get(note1).sampleTimeRange(0.0, timeToWaitUntilBail)
                            );

        scoreFirstPathPart2 = SwervePathBuilder.generate(maxVel_SHOT, maxAcc_SHOT, scorePath.get(note1).sample(timeToWaitUntilBail).velocityMetersPerSecond, 0.0,
                                scorePath.get(note1).sampleTimeRange(timeToWaitUntilBail, scorePath.get(note1).getTotalTimeSeconds())
                            );

        scoreSecondPathPart1 = SwervePathBuilder.generate(maxVel_SHOT, maxAcc_SHOT, 0.0, scorePath.get(note2).sample(timeToWaitUntilBail).velocityMetersPerSecond,
                                scorePath.get(note2).sampleTimeRange(0.0, timeToWaitUntilBail)
                            );
        
        scoreSecondPathPart2 = SwervePathBuilder.generate(maxVel_SHOT, maxAcc_SHOT, scorePath.get(note2).sample(timeToWaitUntilBail).velocityMetersPerSecond, 0.0,
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
            drive(grabFirstPathPart1).deadlineWith(RobotContainer.shootSequence()), // if shot fires in correct time, add RobotContainer.hintAng(0.0, 0.0) or whatever wrist angle may be
            driveAndFetchFixed(grabFirstPathPart2, 0.75),

            driveAndIntake(scoreFirstPathPart1).alongWith(RobotContainer.hintAuton(aimMap.get(this.note1))),

            Commands.either(
                Commands.sequence(
                    driveAndIntake(scoreFirstPathPart2).alongWith(RobotContainer.hintAuton(aimMap.get(this.note1))).withTimeout(scorePath.get(note1).getTotalTimeSeconds() - timeToWaitUntilBail),
                    
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
                    driveAndIntake(scoreSecondPathPart2).alongWith(RobotContainer.hintAuton(aimMap.get(this.note2))).withTimeout(scorePath.get(note2).getTotalTimeSeconds() - timeToWaitUntilBail),
                    
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
            
            driveAndIntake(scorePath.get(this.note3)).alongWith(RobotContainer.hintAuton(aimMap.get(this.note3))).withTimeout(scorePath.get(note3).getTotalTimeSeconds()),

            RobotContainer.autonAdjust(),
            RobotContainer.shootSequence().beforeStarting(RobotContainer.hintAuton(aimMap.get(this.note3), 0.05))
        );
    }

    @Override
    public Route getRoute() {
        return new Route(
            grabFirstPathPart1,
            grabFirstPathPart2,
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