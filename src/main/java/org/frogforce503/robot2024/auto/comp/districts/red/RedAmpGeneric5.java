package org.frogforce503.robot2024.auto.comp.districts.red;

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

/**
 * DO NOTE USE
 */
public class RedAmpGeneric5 extends AutoMode {

    private PlannedPath goOutForFirstButShoot, gobbleC, scoreC;
    private HashMap<CENTERLINE_NOTES, PlannedPath> grabFirstPaths = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> grabSecondPaths = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> scorePaths = new HashMap<>();

    private final double steadyShotDistance = 3.711; // TODO: tune based on the value in the shotmap with 0 wrist 0 arm
    Translation2d firstShotPos;

    double finalHint;

    private CENTERLINE_NOTES note1, note2, note3;

    public RedAmpGeneric5(CENTERLINE_NOTES note1, CENTERLINE_NOTES note2, CENTERLINE_NOTES note3) {

        Pose2d startingPose = setupPose(
            Waypoints.facingGoal(FieldConfig.getInstance().NOTE_C.interpolate(FieldConfig.getInstance().NOTE_B, 0.5).plus(new Translation2d(1.0, 0.0)))
        );

        firstShotPos = FieldConfig.getInstance().RED_SPEAKER.plus(new Translation2d(-steadyShotDistance, 0.75));

        Pose2d firstShotPose = Waypoints.facingGoal(firstShotPos);

        
        goOutForFirstButShoot = SwervePathBuilder.generate(4.0, 3.0, 0.0, 0.675,
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().NOTE_C.interpolate(FieldConfig.getInstance().NOTE_B, 0.5), new Rotation2d(Math.PI), firstShotPose.getRotation()),
            Waypoint.fromHolonomicPose(firstShotPose)
        );

        var grabGFirst = SwervePathBuilder.generate(4.0, 3.0, 0.675, 0.0,
            Waypoint.fromHolonomicPose(goOutForFirstButShoot.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0, 1.0)), null, new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().NOTE_G.plus(new Translation2d(Units.feetToMeters(0.5), -Units.feetToMeters(1.25))))
        );

        var grabFFirst = SwervePathBuilder.generate(4.0, 3.0, 0.675, 0.0,
            Waypoint.fromHolonomicPose(goOutForFirstButShoot.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(-0.5, 1.0)), null, new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(Units.feetToMeters(3.0), Units.feetToMeters(-1.0))), new Rotation2d(Math.PI), new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(-Units.feetToMeters(0.5), Units.feetToMeters(0.5))), new Rotation2d(Math.PI), new Rotation2d(Math.PI))
        );

        var grabHFirst = SwervePathBuilder.generate(4.0, 3.0, 0.675, 0.0,
            Waypoint.fromHolonomicPose(goOutForFirstButShoot.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0, 1.0)), null, new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().NOTE_H.plus(new Translation2d(Units.feetToMeters(0.5), 0)), new Rotation2d(Math.PI), null)
        );

        grabFirstPaths.put(CENTERLINE_NOTES.G, grabGFirst);
        grabFirstPaths.put(CENTERLINE_NOTES.F, grabFFirst);
        grabFirstPaths.put(CENTERLINE_NOTES.H, grabHFirst);

        var comeBackScoreG = SwervePathBuilder.generate(4.0, 3.5,
            Waypoint.fromHolonomicPose(grabGFirst.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0, 1.0)), null, new Rotation2d(Math.PI)),
            Waypoint.fromHolonomicPose(redSourceFarShot.getFirst())
        );

        var comeBackScoreF = SwervePathBuilder.generate(4.0, 3.5,
            Waypoint.fromHolonomicPose(grabFFirst.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(-0.5, 1.0)), null, new Rotation2d(Math.PI)),
            Waypoint.fromHolonomicPose(redSourceFarShot.getFirst())
        );

        var comeBackScoreH = SwervePathBuilder.generate(4.0, 3.5,
            Waypoint.fromHolonomicPose(grabHFirst.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0, 1.0)), null, new Rotation2d(Math.PI)),
            Waypoint.fromHolonomicPose(redSourceFarShot.getFirst())
        );

        scorePaths.put(CENTERLINE_NOTES.G, comeBackScoreG);
        scorePaths.put(CENTERLINE_NOTES.F, comeBackScoreF);
        scorePaths.put(CENTERLINE_NOTES.H, comeBackScoreH);

        var grabGSecond = SwervePathBuilder.generate(4.0, 3.0, 0.675, 0.0,
            Waypoint.fromHolonomicPose(redSourceFarShot.getFirst()),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0, 1.0)), null, new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().NOTE_G.plus(new Translation2d(Units.feetToMeters(1), -Units.feetToMeters(0.6))), new Rotation2d(Math.PI), null)
        );

        var grabFSecond = SwervePathBuilder.generate(4.0, 3.5,
            Waypoint.fromHolonomicPose(redSourceFarShot.getFirst()),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(-0.5, 1.0)), null, new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(Units.feetToMeters(3.0), Units.feetToMeters(-0.5))), new Rotation2d(Math.PI), new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(-Units.feetToMeters(0.5), Units.feetToMeters(-0.5))), new Rotation2d(Math.PI), new Rotation2d(Math.PI))
        );

        var grabHSecond = SwervePathBuilder.generate(4.0, 3.0, 0.675, 0.0,
            Waypoint.fromHolonomicPose(redSourceFarShot.getFirst()),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0, 1.0)), null, new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().NOTE_H.plus(new Translation2d(Units.feetToMeters(1), -Units.feetToMeters(0))))
        );

        grabSecondPaths.put(CENTERLINE_NOTES.G, grabGSecond);
        grabSecondPaths.put(CENTERLINE_NOTES.F, grabFSecond);
        grabSecondPaths.put(CENTERLINE_NOTES.H, grabHSecond);

        // comeBackScoreF = SwervePathBuilder.generate(4.0, 3.5,
        //     Waypoint.fromHolonomicPose(goOutForF.getFinalHolonomicPose()),
        //     new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(-0.5, 1.0)), null, new Rotation2d(Math.PI)),
        //     Waypoint.fromHolonomicPose(redSourceFarShot.getFirst())
        // );

        Rotation2d gobbleCDirection = FieldConfig.getInstance().NOTE_C.minus(redSourceFarShot.getFirst().getTranslation()).getAngle();

        gobbleC = SwervePathBuilder.generate(4.0, 3.5,
            Waypoint.fromHolonomicPose(comeBackScoreF.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().NOTE_C.minus(new Translation2d(0.5, gobbleCDirection)), gobbleCDirection, gobbleCDirection),
            new Waypoint(FieldConfig.getInstance().NOTE_C, gobbleCDirection, gobbleCDirection)
        );

        scoreC = SwervePathBuilder.generate(4.0, 3.5,
            Waypoint.fromHolonomicPose(gobbleC.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().NOTE_C.plus(new Translation2d(1.0, 0)))
            // new Waypoint(FieldConfig.getInstance().RED_SPEAKER.plus())
        );

        finalHint = ShotPlanner.getInstance().getRawDistanceToGoal(scoreC.getFinalHolonomicPose().getTranslation()) + Units.feetToMeters(1);

        this.note1 = note1;
        this.note2 = note2;
    }

    @Override
    public CENTERLINE_NOTES[] priorityList() {
        return new CENTERLINE_NOTES[] { note1, note2 };
    }

    @Override
    public Command routine() {
        return Commands.sequence(
            RobotContainer.shooter.rampUp(),
            Commands.deadline( // grabs G while shooting P
                Commands.sequence(
                    drive(goOutForFirstButShoot, () -> RobotContainer.drive.getPose().getTranslation().getX() <= FieldConfig.getInstance().NOTE_B.getX() + 0.5 && RobotContainer.feeder.noteInEntrySensor()),
                    drive(grabFirstPaths.get(this.note1)).fetchAtEndFor(1.0).withTimeout(grabFirstPaths.get(this.note1).getTotalTimeSeconds() - 0.5)
                ),
                Commands.waitUntil(() -> RobotContainer.drive.getPose().getTranslation().getX() <= firstShotPos.getX() + Units.feetToMeters(0.5))
                    .deadlineWith(RobotContainer.hint(steadyShotDistance + Units.feetToMeters(6))/*, RobotContainer.hintAng(arm, wrist) */)
                    .andThen(
                        RobotContainer.shootSequence(),
                        RobotContainer.intake().alongWith(RobotContainer.wrist.intakeAssist())
                    )
            ),
            driveAndIntake(scorePaths.get(this.note1), () -> true)
                .deadlineWith(
                    RobotContainer.hintAng(redSourceFarShot.getSecond(), redSourceFarShot.getThird()),
                    Commands.print("RUNNING BACK PATH").repeatedly()
                ),
            RobotContainer.shootSequence(),
            drive(grabSecondPaths.get(this.note2)).fetchAtEndFor(1.0).deadlineWith(RobotContainer.wrist.intakeAssist()).withTimeout(grabSecondPaths.get(this.note2).getTotalTimeSeconds()),
            driveAndIntake(scorePaths.get(this.note2), () -> true)
                .deadlineWith(RobotContainer.hintAng(redSourceFarShot.getSecond(), redSourceFarShot.getThird())),
            RobotContainer.shootSequence(),
            driveAndIntake(gobbleC).deadlineWith(RobotContainer.hint(finalHint)),
            driveAndIntake(scoreC, () -> true),
            Commands.waitSeconds(0.5),
            RobotContainer.shootSequence()
        );
    }

    @Override
    public Route getRoute() {
        return new Route()
            .addPaths(goOutForFirstButShoot, grabFirstPaths.get(this.note1), scorePaths.get(this.note1), grabSecondPaths.get(this.note2), scorePaths.get(this.note2), gobbleC, scoreC);
    }
    
}
