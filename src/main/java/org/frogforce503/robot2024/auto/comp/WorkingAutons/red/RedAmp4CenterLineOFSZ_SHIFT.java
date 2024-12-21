/*
 * Shoots preload on the move then picks up and shoot 3 ceterline notes
 */


package org.frogforce503.robot2024.auto.comp.WorkingAutons.red;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import java.lang.reflect.Field;
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

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RedAmp4CenterLineOFSZ_SHIFT extends AutoMode {

    private PlannedPath goOutForFirstButShoot;
    private HashMap<CENTERLINE_NOTES, PlannedPath> grabFirstPaths = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> grabSecondPaths = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> scorePaths = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, Pair<Double, Double>> aimMap = new HashMap<>();

    PlannedPath driveFtoG, driveFtoH, driveGtoF, driveGtoH, driveHtoF, driveHtoG;

    private HashMap<CENTERLINE_NOTES, PlannedPath> shiftFromFPath = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> shiftFromGPath = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> shiftFromHPath = new HashMap<>();

    private HashMap<CENTERLINE_NOTES, HashMap<CENTERLINE_NOTES, PlannedPath>> shiftPath = new HashMap<>();

    PlannedPath scoreFirstPathPart1, scoreFirstPathPart2, scoreSecondPathPart1, scoreSecondPathPart2;

    private final double steadyShotDistance = 3.711;

    private double maxVel = 4.6;
    private double maxAcc = 4.75;
    
    Translation2d firstShotPos;
    Translation2d secondShotPos;

    private CENTERLINE_NOTES note1, note2, note3;

    public RedAmp4CenterLineOFSZ_SHIFT(CENTERLINE_NOTES note1, CENTERLINE_NOTES note2, CENTERLINE_NOTES note3) {

        Pose2d startingPose = setupPose(
            Waypoints.facingGoal(FieldConfig.getInstance().NOTE_C.interpolate(FieldConfig.getInstance().NOTE_B, 0.5).plus(new Translation2d(1.0, 0.0)))
        );

        firstShotPos = FieldConfig.getInstance().RED_SPEAKER.minus(new Translation2d(steadyShotDistance, -0.75));
        Pose2d firstShotPose = Waypoints.facingGoal(firstShotPos);
        // Pose2d firstShotPose = new Pose2d(firstShotPos, new Rotation2d(Math.PI).plus(Rotation2d.fromDegrees(-7)));

        secondShotPos = firstShotPos.minus(new Translation2d(0.5, 0));
        Pose2d secondShotPose = Waypoints.facingGoal(secondShotPos);

        aimMap.put(CENTERLINE_NOTES.F, Pair.of(0.0, 0.25));
        aimMap.put(CENTERLINE_NOTES.G, Pair.of(0.0, 0.25));
        aimMap.put(CENTERLINE_NOTES.H, Pair.of(0.0, 0.25));

        goOutForFirstButShoot = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().NOTE_C.interpolate(FieldConfig.getInstance().NOTE_B, 0.5), new Rotation2d(Math.PI), firstShotPose.getRotation()),
            Waypoint.fromHolonomicPose(firstShotPose/*.transformBy(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(3)))*/)
        );

        var grabGFirst = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(goOutForFirstButShoot.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0, 1.0)), null, new Rotation2d(Math.PI).plus(Rotation2d.fromDegrees(20))),
            new Waypoint(FieldConfig.getInstance().NOTE_G, null, new Rotation2d(Math.PI).plus(Rotation2d.fromDegrees(20)))
        );

        var grabFFirst = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(goOutForFirstButShoot.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_LEFT.interpolate(FieldConfig.getInstance().RED_STAGE_RIGHT, 0.5).interpolate(FieldConfig.getInstance().RED_CENTER_STAGE, 0.5).plus(new Translation2d(0,Units.feetToMeters(1.0
            ))), null, new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().NOTE_F, null, new Rotation2d(Math.PI))
        );

        
        
        Rotation2d temp = FieldConfig.getInstance().NOTE_H.minus(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0, 1.625))).getAngle();

        var grabHFirst = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(goOutForFirstButShoot.getFinalHolonomicPose()),
            // new Waypoint(goOutForFirstButShoot.getFinalHolonomicPose().getTranslation().plus(new Translation2d(Units.feetToMeters(-1), Units.feetToMeters(1))), null, Rotation2d.fromDegrees(150)),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0, 1.5)), null, temp.plus(Rotation2d.fromDegrees(-40       
            ))),
            new Waypoint(FieldConfig.getInstance().NOTE_H, null, temp)
        );

        grabFirstPaths.put(CENTERLINE_NOTES.G, grabGFirst);
        grabFirstPaths.put(CENTERLINE_NOTES.F, grabFFirst);
        grabFirstPaths.put(CENTERLINE_NOTES.H, grabHFirst);


        var grabGSecond = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(secondShotPose),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0, 1.0)), null, new Rotation2d(Math.PI).plus(Rotation2d.fromDegrees(10))),
            new Waypoint(FieldConfig.getInstance().NOTE_G, null, new Rotation2d(Math.PI).plus(Rotation2d.fromDegrees(10)))
        );

        // var grabFSecond = SwervePathBuilder.generate(maxVel, maxAcc,
        //     Waypoint.fromHolonomicPose(secondShotPose),
        //     new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0.5, 1.0)), null, new Rotation2d()),
        //     new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(Units.feetToMeters(-3.0), Units.feetToMeters(-0.5))), new Rotation2d(), new Rotation2d()),
        //     new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(Units.feetToMeters(0.5), Units.feetToMeters(-0.5))), new Rotation2d(), new Rotation2d())
        // );

        var grabFSecond = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(secondShotPose),
            // new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.interpolate(FieldConfig.getInstance().RED_CENTER_STAGE, 0.5), null, Rotation2d.fromDegrees(240)),
            // new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.interpolate(FieldConfig.getInstance().RED_STAGE_LEFT, 0.5), null, new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_LEFT.interpolate(FieldConfig.getInstance().RED_STAGE_RIGHT, 0.5).interpolate(FieldConfig.getInstance().RED_CENTER_STAGE, 0.5).plus(new Translation2d(0,Units.feetToMeters(1.0
            ))), null, new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().NOTE_F, null, new Rotation2d(Math.PI))
        );

        var grabHSecond = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(secondShotPose),
            // new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0, 1.5)), null, new Rotation2d(Math.PI).plus(Rotation2d.fromDegrees(-10))),
            new Waypoint(FieldConfig.getInstance().NOTE_H, null, new Rotation2d(Math.PI).plus(Rotation2d.fromDegrees(-10)))
        );

        grabSecondPaths.put(CENTERLINE_NOTES.G, grabGSecond);
        grabSecondPaths.put(CENTERLINE_NOTES.F, grabFSecond);
        grabSecondPaths.put(CENTERLINE_NOTES.H, grabHSecond);


        var comeBackScoreG = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(grabGFirst.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0, 1.25)), null, secondShotPose.getRotation().plus(Rotation2d.fromDegrees(-3))),
            Waypoint.fromHolonomicPose(secondShotPose/*.transformBy(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(3)))*/)
        );

        var comeBackScoreF = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(grabFSecond.getFinalHolonomicPose()),
            // new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0.5, 1.0)), null, new Rotation2d()),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_LEFT.interpolate(FieldConfig.getInstance().RED_STAGE_RIGHT, 0.5).interpolate(FieldConfig.getInstance().RED_CENTER_STAGE, 0.5).plus(new Translation2d(0,Units.feetToMeters(1.0
            ))), null, secondShotPose.getRotation().plus(Rotation2d.fromDegrees(-5))),
            Waypoint.fromHolonomicPose(secondShotPose/*.transformBy(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(3)))*/) // 3.5
        );

        var comeBackScoreH = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(grabHFirst.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0, 1.5)), null, temp.plus(Rotation2d.fromDegrees(-5       
            ))),
            Waypoint.fromHolonomicPose(secondShotPose/*.transformBy(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(3)))*/)
        );

        scorePaths.put(CENTERLINE_NOTES.G, comeBackScoreG);
        scorePaths.put(CENTERLINE_NOTES.F, comeBackScoreF);
        scorePaths.put(CENTERLINE_NOTES.H, comeBackScoreH);

        // comeBackScoreF = SwervePathBuilder.generate(maxVel, maxAcc,
        //     Waypoint.fromHolonomicPose(goOutForF.getFinalHolonomicPose()),
        //     new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(-0.5, 1.0)), null, new Rotation2d(Math.PI)),
        //     Waypoint.fromHolonomicPose(secondShotPose)
        // );


        this.note1 = note1;
        this.note2 = note2;
        this.note3 = note3;

        driveFtoG = SwervePathBuilder.generate(4.0, 3.5, 
            Waypoint.fromPlannedPathState(comeBackScoreF.sample(1.0)),
            new Waypoint(grabGFirst.getFinalHolonomicPose().getTranslation(), null, new Rotation2d(Math.PI).plus(Rotation2d.fromDegrees(-20)))
        );

        driveFtoH = SwervePathBuilder.generate(4.0, 3.5, 
            Waypoint.fromPlannedPathState(comeBackScoreF.sample(1.0)),
            new Waypoint(grabHFirst.getFinalHolonomicPose().getTranslation(), null, new Rotation2d(Math.PI).plus(Rotation2d.fromDegrees(-30)))
        );

        driveGtoF = SwervePathBuilder.generate(4.0, 3.5, 
            Waypoint.fromPlannedPathState(comeBackScoreG.sample(1.0)),
            new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(1.75, 0.5)), null, new Rotation2d(Math.PI)),
            new Waypoint(grabFFirst.getFinalHolonomicPose().getTranslation(), null, new Rotation2d(Math.PI))
        );

        driveGtoH = SwervePathBuilder.generate(4.0, 3.5, 
            Waypoint.fromPlannedPathState(comeBackScoreG.sample(1.0)),
            new Waypoint(grabHFirst.getFinalHolonomicPose().getTranslation(), null, new Rotation2d(Math.PI))
        );

        driveHtoF = SwervePathBuilder.generate(4.0, 3.5, 
            Waypoint.fromPlannedPathState(comeBackScoreH.sample(1.0)),
            new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(1.75, 0.5)), null, new Rotation2d(Math.PI)),
            new Waypoint(grabFFirst.getFinalHolonomicPose().getTranslation(), null, new Rotation2d(Math.PI))
        );

        driveHtoG = SwervePathBuilder.generate(4.0, 3.5, 
            Waypoint.fromPlannedPathState(comeBackScoreH.sample(1.0)),
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

        scoreFirstPathPart1 = SwervePathBuilder.generate(maxVel, maxAcc, 0.0, scorePaths.get(note1).sample(1.0).velocityMetersPerSecond,
                                Waypoint.fromHolonomicPose(scorePaths.get(note1).getInitialHolonomicPose()),
                                Waypoint.fromPlannedPathState(scorePaths.get(note1).sample(1.0))
                            );

        scoreFirstPathPart2 = SwervePathBuilder.generate(maxVel, maxAcc, scorePaths.get(note1).sample(1.0).velocityMetersPerSecond, 0.0,
                                Waypoint.fromPlannedPathState(scorePaths.get(note1).sample(1.0)),
                                Waypoint.fromHolonomicPose(scorePaths.get(note1).getFinalHolonomicPose())
                            );

        scoreSecondPathPart1 = SwervePathBuilder.generate(maxVel, maxAcc, 0.0, scorePaths.get(note2).sample(1.0).velocityMetersPerSecond,
                                Waypoint.fromHolonomicPose(scorePaths.get(note2).getInitialHolonomicPose()),
                                Waypoint.fromPlannedPathState(scorePaths.get(note2).sample(1.0))
                            );
        
        scoreSecondPathPart2 = SwervePathBuilder.generate(maxVel, maxAcc, scorePaths.get(note2).sample(1.0).velocityMetersPerSecond, 0.0,
                                Waypoint.fromPlannedPathState(scorePaths.get(note2).sample(1.0)),
                                Waypoint.fromHolonomicPose(scorePaths.get(note2).getFinalHolonomicPose())
                            );
    }

    @Override
    public CENTERLINE_NOTES[] priorityList() {
        return new CENTERLINE_NOTES[] { note1, note2, note3 };
    }

    @Override
    public Command routine() {
        return Commands.sequence(
            drive(goOutForFirstButShoot).alongWith(RobotContainer.hintAng(0, 0.65), RobotContainer.shooter.rampUp()).withTimeout(goOutForFirstButShoot.getTotalTimeSeconds()),
            RobotContainer.shootSequence().beforeStarting(Commands.parallel(RobotContainer.shooter.rampUp(), RobotContainer.hintAng(0, 0.65)).withTimeout(0.1)),

            driveAndFetchFixed(grabFirstPaths.get(this.note1), 0.7),

            driveAndIntake(scoreFirstPathPart1),

            Commands.either(
                Commands.sequence(
                    driveAndIntake(scoreFirstPathPart2).alongWith(RobotContainer.shooter.rampUp(), RobotContainer.hintAng(aimMap.get(this.note1))).withTimeout(scorePaths.get(note1).getTotalTimeSeconds() - 1 - 0.35),
                    
                    RobotContainer.shootSequence().beforeStarting(Commands.parallel(RobotContainer.shooter.rampUp(), RobotContainer.hintAng(aimMap.get(this.note1))).withTimeout(0.1)),

                    RobotContainer.hintAng(aimMap.get(this.note2)),
                    driveAndFetchFixed(grabSecondPaths.get(this.note2), 0.7)
                ),
                Commands.sequence(
                    RobotContainer.hintAng(aimMap.get(this.note2)),
                    driveAndIntake(shiftPath.get(this.note1).get(this.note2))   
                ),
                RobotContainer.feeder::noteInEntrySensor
            ),

            driveAndIntake(scoreSecondPathPart1),

            Commands.either(
                Commands.sequence(
                    driveAndIntake(scoreSecondPathPart2).alongWith(RobotContainer.hintAng(aimMap.get(this.note2))).withTimeout(scorePaths.get(note2).getTotalTimeSeconds() - 1 - 0.1),
                    
                    RobotContainer.shootSequence().beforeStarting(Commands.parallel(RobotContainer.shooter.rampUp(), RobotContainer.hintAng(aimMap.get(this.note2))).withTimeout(0.1)),

                    RobotContainer.hintAng(aimMap.get(this.note3)),
                    driveAndFetchFixed(grabSecondPaths.get(this.note3), 0.7)
                ),
                Commands.sequence(
                    RobotContainer.hintAng(aimMap.get(this.note3)),
                    driveAndIntake(shiftPath.get(this.note2).get(this.note3))
                ),
                RobotContainer.feeder::noteInEntrySensor
            ),
            
            driveAndIntake(scorePaths.get(this.note3)).alongWith(RobotContainer.hintAng(aimMap.get(this.note3))).withTimeout(scorePaths.get(note3).getTotalTimeSeconds() - 0.1),
            RobotContainer.shootSequence().beforeStarting(Commands.parallel(RobotContainer.shooter.rampUp(), RobotContainer.hintAng(aimMap.get(this.note3))).withTimeout(0.1))
        );
    }

    @Override
    public Route getRoute() {
        return new Route()
            .addPaths(goOutForFirstButShoot, grabFirstPaths.get(this.note1), scorePaths.get(this.note1), grabSecondPaths.get(this.note2), scorePaths.get(this.note2), grabSecondPaths.get(this.note3), scorePaths.get(this.note3));
    }
    
}
