package org.frogforce503.robot2024.auto.comp.districts.red;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import java.util.HashMap;

// import static edu.wpi.first.units.Units.Rotation;

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

public class RedAmpGeneric5Cleanup extends AutoMode {

    private PlannedPath goOutForFirstButShoot, gobbleC, gobbleB;
    private HashMap<CENTERLINE_NOTES, PlannedPath> grabFirstPaths = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> grabSecondPaths = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> scorePaths = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, Pair<Double, Double>> aimMap = new HashMap<>();

    private final double steadyShotDistance = 3.711; // TODO: tune based on the value in the shotmap with 0 wrist 0 arm

    private double maxVel = 4.0;
    private double maxAcc = 3.5;
    
    Translation2d firstShotPos;
    Translation2d secondShotPos;

    private CENTERLINE_NOTES note1, note2;

    public RedAmpGeneric5Cleanup(CENTERLINE_NOTES note1, CENTERLINE_NOTES note2) {

        Pose2d startingPose = setupPose(
            Waypoints.facingGoal(FieldConfig.getInstance().NOTE_C.interpolate(FieldConfig.getInstance().NOTE_B, 0.5).plus(new Translation2d(1.0, 0.0)))
        );

        firstShotPos = FieldConfig.getInstance().RED_SPEAKER.plus(new Translation2d(-steadyShotDistance, 0.75));
        Pose2d firstShotPose = Waypoints.facingGoal(firstShotPos).transformBy(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(5)));

        
        secondShotPos = firstShotPos.plus(new Translation2d(-0.374, 0)); // redSourceFarShot.getFirst().getTranslation();
        Pose2d secondShotPose = Waypoints.facingGoal(secondShotPos); // redSourceFarShot.getFirst();


        aimMap.put(CENTERLINE_NOTES.F, Pair.of(0.75, 0.0));
        aimMap.put(CENTERLINE_NOTES.G, Pair.of(0.75, 0.0));
        aimMap.put(CENTERLINE_NOTES.H, Pair.of(3.0, 0.0));

        
        goOutForFirstButShoot = SwervePathBuilder.generate(4.0, 3.0, 0.0, 0.675,
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().NOTE_C.interpolate(FieldConfig.getInstance().NOTE_B, 0.5), new Rotation2d(Math.PI), firstShotPose.getRotation()),
            Waypoint.fromHolonomicPose(firstShotPose)
        );

        var grabGFirst = SwervePathBuilder.generate(maxVel, maxAcc, 0.675, 0.0,
            Waypoint.fromHolonomicPose(goOutForFirstButShoot.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0, 1.0)), null, new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().NOTE_G.plus(new Translation2d(Units.feetToMeters(0.5), -Units.feetToMeters(1.25))))
        );

        var grabFFirst = SwervePathBuilder.generate(maxVel, maxAcc, 0.675, 0.0,
            Waypoint.fromHolonomicPose(goOutForFirstButShoot.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(-0.5, 1.0)), null, new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(Units.feetToMeters(3.0), Units.feetToMeters(-1.0))), new Rotation2d(Math.PI), new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(-Units.feetToMeters(0.5), Units.feetToMeters(0.5))), new Rotation2d(Math.PI), new Rotation2d(Math.PI))
        );

        var grabHFirst = SwervePathBuilder.generate(maxVel, maxAcc, 0.675, 0.0,
            Waypoint.fromHolonomicPose(goOutForFirstButShoot.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0, 1.0)), null, new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().NOTE_H.plus(new Translation2d(Units.feetToMeters(0.5), 0)), new Rotation2d(Math.PI), null)
        );

        grabFirstPaths.put(CENTERLINE_NOTES.G, grabGFirst);
        grabFirstPaths.put(CENTERLINE_NOTES.F, grabFFirst);
        grabFirstPaths.put(CENTERLINE_NOTES.H, grabHFirst);

        var comeBackScoreG = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(grabGFirst.getFinalHolonomicPose()),
            // new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0, 1.0)), null, new Rotation2d(Math.PI)),
            Waypoint.fromHolonomicPose(secondShotPose)
        );

        var comeBackScoreF = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(grabFFirst.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(-0.5, 1.0)), null, new Rotation2d(Math.PI)),
            Waypoint.fromHolonomicPose(secondShotPose.transformBy(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(10))))
        );

        var comeBackScoreH = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(grabHFirst.getFinalHolonomicPose()),
            // new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0, 1.0)), null, new Rotation2d(Math.PI)),
            Waypoint.fromHolonomicPose(secondShotPose.transformBy(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(10))))
        );

        scorePaths.put(CENTERLINE_NOTES.G, comeBackScoreG);
        scorePaths.put(CENTERLINE_NOTES.F, comeBackScoreF);
        scorePaths.put(CENTERLINE_NOTES.H, comeBackScoreH);

        var grabGSecond = SwervePathBuilder.generate(maxVel, maxAcc, 0.675, 0.0,
            Waypoint.fromHolonomicPose(secondShotPose),
            // new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0, 1.0)), null, new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().NOTE_G.plus(new Translation2d(Units.feetToMeters(1), -Units.feetToMeters(0.6))), new Rotation2d(Math.PI), new Rotation2d(Math.PI))
        );

        var grabFSecond = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(secondShotPose.transformBy(new Transform2d(new Translation2d(-0.5, 0), new Rotation2d()))),
            // new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(-0.5, 1.0)), null, new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(Units.feetToMeters(3.0), Units.feetToMeters(-0.5))), new Rotation2d(Math.PI), new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(-Units.feetToMeters(0.5), Units.feetToMeters(-0.5))), new Rotation2d(Math.PI), new Rotation2d(Math.PI))
        );

        var grabHSecond = SwervePathBuilder.generate(maxVel, maxAcc, 0.675, 0.0,
            Waypoint.fromHolonomicPose(secondShotPose),
            // new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0, 1.0)), null, new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().NOTE_H.plus(new Translation2d(Units.feetToMeters(1), -Units.feetToMeters(0))))
        );

        grabSecondPaths.put(CENTERLINE_NOTES.G, grabGSecond);
        grabSecondPaths.put(CENTERLINE_NOTES.F, grabFSecond);
        grabSecondPaths.put(CENTERLINE_NOTES.H, grabHSecond);


        Rotation2d gobbleCDirection = FieldConfig.getInstance().NOTE_C.minus(secondShotPos).getAngle();

        gobbleC = SwervePathBuilder.generate(4.0, 3.5, 0, 0,
            Waypoint.fromHolonomicPose(secondShotPose),
            new Waypoint(FieldConfig.getInstance().NOTE_C.minus(new Translation2d(1, gobbleCDirection)), gobbleCDirection, gobbleCDirection),
            new Waypoint(FieldConfig.getInstance().NOTE_C, gobbleCDirection, gobbleCDirection),
            // new Waypoint(FieldConfig.getInstance().NOTE_C.plus(new Translation2d(1.0, 0))),
            new Waypoint(FieldConfig.getInstance().NOTE_B.interpolate(FieldConfig.getInstance().NOTE_C, 0.5).plus(new Translation2d(0.7, 0))),
            new Waypoint(FieldConfig.getInstance().NOTE_B.plus(new Translation2d(1.4, 0)), null, new Rotation2d(Math.PI))
        );

        gobbleB = SwervePathBuilder.generate(4.0, 3.5, 0, 0,
            Waypoint.fromHolonomicPose(gobbleC.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().NOTE_B)
        );


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
                // FIRST SHOT AND FIRST CENTER LINE PICKUP
                Commands.sequence(
                    drive(goOutForFirstButShoot),
                    drive(grabFirstPaths.get(this.note1)).fetchAtEndFor(1.0).withTimeout(grabFirstPaths.get(this.note1).getTotalTimeSeconds() - 0.5)
                ),
                Commands.waitUntil(() -> RobotContainer.drive.getPose().getTranslation().getX() <= firstShotPos.getX() + Units.feetToMeters(2))
                    .deadlineWith(RobotContainer.hintAng(0, 0))
                    .andThen(
                        RobotContainer.shootSequence(),
                        RobotContainer.intake()
                    )
            ),
            // COME BACK SCORE SECOND NOTE
            RobotContainer.hintAng(aimMap.get(this.note1)), // 3, 0
            driveAndIntake(scorePaths.get(this.note1))
                .deadlineWith(
                    RobotContainer.startAim(),
                    Commands.print("RUNNING BACK PATH").repeatedly()
                ),
            RobotContainer.shootSequence(),
            // GRAB SECOND CENTER LINE NOTE
            driveAndFetchFixed(grabSecondPaths.get(this.note2), 1.5).withTimeout(grabSecondPaths.get(this.note2).getTotalTimeSeconds()),
            driveAndIntake(scorePaths.get(this.note2))
            .deadlineWith(RobotContainer.hintAng(aimMap.get(this.note2)).alongWith(RobotContainer.startAim())), // 0.75, 0
            RobotContainer.shootSequence(),
            // GRAB FOURTH NOTE
            driveAndIntake(gobbleC).deadlineWith(RobotContainer.hintAng(0, 19), RobotContainer.startAim().beforeStarting(waitUntil(() -> RobotContainer.feeder.noteInExitSensor()))),
            RobotContainer.shootSequence(),
            // GRAB FIFTH NOTE
            driveAndIntake(gobbleB).deadlineWith(RobotContainer.hintAng(0, 5), RobotContainer.startAim().beforeStarting(waitSeconds(0.075))),
            RobotContainer.shootSequence()
        );
    }

    @Override
    public Route getRoute() {
        return new Route()
            .addPaths(goOutForFirstButShoot, grabFirstPaths.get(this.note1), scorePaths.get(this.note1), grabSecondPaths.get(this.note2), scorePaths.get(this.note2), gobbleC, gobbleB);
    }
    
}
