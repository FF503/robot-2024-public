package org.frogforce503.robot2024.auto.comp.districts.red;

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

public class RedAmpGeneric6Cleanup extends AutoMode {

    private PlannedPath goOutForFirstButShoot, gobbleC, gobbleB;
    private HashMap<CENTERLINE_NOTES, PlannedPath> grabFirstPaths = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> grabSecondPaths = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> scorePaths = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, Pair<Double, Double>> aimMap = new HashMap<>();

    private final double steadyShotDistance = 3.711; // TODO: tune based on the value in the shotmap with 0 wrist 0 arm

    private double maxVel = 5.5;
    private double maxAcc = 4.75;
    
    Translation2d firstShotPos;
    Translation2d secondShotPos;

    private CENTERLINE_NOTES note1, note2, note3;

    public RedAmpGeneric6Cleanup(CENTERLINE_NOTES note1, CENTERLINE_NOTES note2, CENTERLINE_NOTES note3) {

        Pose2d startingPose = setupPose(
            Waypoints.facingGoal(FieldConfig.getInstance().NOTE_C.interpolate(FieldConfig.getInstance().NOTE_B, 0.5).plus(new Translation2d(1.0, 0.0)))
        );

        firstShotPos = FieldConfig.getInstance().RED_SPEAKER.plus(new Translation2d(-steadyShotDistance, 0.75));

        Pose2d firstShotPose = Waypoints.facingGoal(firstShotPos).transformBy(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(5)));

        
        secondShotPos = redSourceFarShot.getFirst().getTranslation(); // firstShotPos.plus(new Translation2d(-0.374, 0));
        Pose2d secondShotPose = redSourceFarShot.getFirst(); // Waypoints.facingGoal(secondShotPos);


        aimMap.put(CENTERLINE_NOTES.F, Pair.of(2.0, 0.0));
        aimMap.put(CENTERLINE_NOTES.G, Pair.of(2.0, 0.0));
        aimMap.put(CENTERLINE_NOTES.H, Pair.of(4.75, 0.0));

        
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
            new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(Units.feetToMeters(3.0), 0)), new Rotation2d(Math.PI), new Rotation2d(Math.PI)),
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

        
        var grabGSecond = SwervePathBuilder.generate(maxVel, maxAcc, 0.675, 0.0,
            Waypoint.fromHolonomicPose(secondShotPose),
            // new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0, 1.0)), null, new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().NOTE_G.plus(new Translation2d(Units.feetToMeters(0), -Units.feetToMeters(0.0))), new Rotation2d(Math.PI), new Rotation2d(Math.PI))
        );

        var grabFSecond = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(secondShotPose/*.transformBy(new Transform2d(new Translation2d(-0.5, 0), new Rotation2d()))*/),
            // new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(-0.5, 1.0)), null, new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(Units.feetToMeters(1.0), Units.feetToMeters(0.0))), new Rotation2d(Math.PI), new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(-Units.feetToMeters(0.5), Units.feetToMeters(0.0))), new Rotation2d(Math.PI), new Rotation2d(Math.PI))
        );

        var grabHSecond = SwervePathBuilder.generate(maxVel, maxAcc, 0.675, 0.0,
            Waypoint.fromHolonomicPose(secondShotPose),
            // new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0, 1.0)), null, new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().NOTE_H.plus(new Translation2d(Units.feetToMeters(1), -Units.feetToMeters(0))))
        );

        grabSecondPaths.put(CENTERLINE_NOTES.G, grabGSecond);
        grabSecondPaths.put(CENTERLINE_NOTES.F, grabFSecond);
        grabSecondPaths.put(CENTERLINE_NOTES.H, grabHSecond);


        var comeBackScoreG = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(grabGSecond.getFinalHolonomicPose()),
            // new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0, 1.0)), null, new Rotation2d(Math.PI)),
            Waypoint.fromHolonomicPose(secondShotPose)
        );

        var comeBackScoreF = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(grabFSecond.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(-0.5, 1.0)), null, new Rotation2d(Math.PI)),
            Waypoint.fromHolonomicPose(secondShotPose.transformBy(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(10))))
        );

        var comeBackScoreH = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(grabHFirst.getFinalHolonomicPose()),
            // new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0, 1.0)), null, new Rotation2d(Math.PI)),
            Waypoint.fromHolonomicPose(secondShotPose.transformBy(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(48.5))))
        );

        scorePaths.put(CENTERLINE_NOTES.G, comeBackScoreG);
        scorePaths.put(CENTERLINE_NOTES.F, comeBackScoreF);
        scorePaths.put(CENTERLINE_NOTES.H, comeBackScoreH);



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
        this.note3 = note3;
    }

    @Override
    public CENTERLINE_NOTES[] priorityList() {
        return new CENTERLINE_NOTES[] { note1, note2, note3 };
    }

    @Override
    public Command routine() {
        return Commands.sequence(
            RobotContainer.shooter.rampUp(),
            Commands.deadline( // grabs G while shooting P
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
            // COME BACK AND SCORE 2ND NOTE
            RobotContainer.hintAng(0, 0),
            driveAndIntake(scorePaths.get(this.note1), () -> true)
                .deadlineWith(
                    RobotContainer.hintAng(aimMap.get(this.note1)), // 4.75, 0
                    Commands.print("RUNNING BACK PATH").repeatedly()
                ),
            RobotContainer.shootSequence(),
            // GRAB 3RD NOTE
            driveAndFetchFixed(grabSecondPaths.get(this.note2), 1.5).withTimeout(grabSecondPaths.get(this.note2).getTotalTimeSeconds()),
            RobotContainer.hintAng(0, 0),
            driveAndIntake(scorePaths.get(this.note2), () -> true)
                .deadlineWith(RobotContainer.hintAng(aimMap.get(this.note2))), // 2, 0
            RobotContainer.shootSequence(),
            // GRAB 4TH NOTE
            driveAndFetchFixed(grabSecondPaths.get(this.note3), 1.5).withTimeout(grabSecondPaths.get(this.note3).getTotalTimeSeconds()),
            RobotContainer.hintAng(0, 0),
            driveAndIntake(scorePaths.get(this.note3), () -> true)
                .deadlineWith(RobotContainer.hintAng(aimMap.get(this.note3))), // 2, 0
            RobotContainer.shootSequence(),
            // GOBBLE 5TH NOTE
            driveAndIntake(gobbleC).deadlineWith(RobotContainer.hintAng(0, 18)),
            RobotContainer.shootSequence(),
            // GOBBLE 6TH NOTE
            driveAndIntake(gobbleB).deadlineWith(RobotContainer.hintAng(0, 5)),
            RobotContainer.shootSequence()
        );
    }

    @Override
    public Route getRoute() {
        return new Route()
            .addPaths(goOutForFirstButShoot, grabFirstPaths.get(this.note1), scorePaths.get(this.note1), grabSecondPaths.get(this.note2), scorePaths.get(this.note2), grabSecondPaths.get(this.note3), scorePaths.get(this.note3), gobbleC, gobbleB);
    }
    
}
