package org.frogforce503.robot2024.auto.comp.districts.blue;

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
import org.frogforce503.robot2024.planners.ShotPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.EllipticalRegionConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class BlueAmpGeneric4 extends AutoMode {

    private PlannedPath goOutForFirstButShoot, gobbleK, scoreK;
    private HashMap<CENTERLINE_NOTES, PlannedPath> grabFirstPaths = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> grabSecondPaths = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> scorePaths = new HashMap<>();

    private final double steadyShotDistance = 3.711; // TODO: tune based on the value in the shotmap with 0 wrist 0 arm
    Translation2d firstShotPos;

    Translation2d secondShotPos;

    double nextShotsHint, finalHint;

    private CENTERLINE_NOTES note1, note2;

    public BlueAmpGeneric4(CENTERLINE_NOTES note1, CENTERLINE_NOTES note2) {

        Pose2d startingPose = setupPose(
            Waypoints.facingGoal(FieldConfig.getInstance().NOTE_K.interpolate(FieldConfig.getInstance().NOTE_J, 0.5).minus(new Translation2d(1.0, 0.0)))
        );

        firstShotPos = FieldConfig.getInstance().BLUE_SPEAKER.plus(new Translation2d(steadyShotDistance, 0.75));

        Pose2d firstShotPose = Waypoints.facingGoal(firstShotPos);

        secondShotPos = firstShotPos.plus(new Translation2d(0.5, 0));
        nextShotsHint = ShotPlanner.getInstance().getRawDistanceToGoal(secondShotPos) + Units.feetToMeters(1.5);

        Pose2d secondShotPose = Waypoints.facingGoal(secondShotPos);

        // TrajectoryConstraint slowDownWhenShooting = new RectangularRegionConstraint(
        //     firstShotPos.plus(new Translation2d(-0.5, -0.5)), 
        //     firstShotPos.plus(new Translation2d(0.5, 0.5)), 
        //     new MaxVelocityConstraint(0.25)
        // );
        
        goOutForFirstButShoot = SwervePathBuilder.generate(4.0, 3.0, 0.0, 0.675,
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().NOTE_K.interpolate(FieldConfig.getInstance().NOTE_J, 0.5), new Rotation2d(), firstShotPose.getRotation()),
            Waypoint.fromHolonomicPose(firstShotPose)
        );

        var grabGFirst = SwervePathBuilder.generate(4.0, 3.0, 0.675, 0.0,
            Waypoint.fromHolonomicPose(goOutForFirstButShoot.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_LEFT.plus(new Translation2d(0, 1.0)), null, new Rotation2d()),
            new Waypoint(FieldConfig.getInstance().NOTE_G.plus(new Translation2d(Units.feetToMeters(-1), -Units.feetToMeters(1.25))))
        );

        var grabFFirst = SwervePathBuilder.generate(4.0, 3.0, 0.675, 0.0,
            Waypoint.fromHolonomicPose(goOutForFirstButShoot.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_LEFT.plus(new Translation2d(0.5, 1.0)), null, new Rotation2d()),
            new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(Units.feetToMeters(-3.0), Units.feetToMeters(-1.0))), new Rotation2d(), new Rotation2d()),
            new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(Units.feetToMeters(0.5), Units.feetToMeters(0.5))), new Rotation2d(), new Rotation2d())
        );

        var grabHFirst = SwervePathBuilder.generate(4.0, 3.0, 0.675, 0.0,
            Waypoint.fromHolonomicPose(goOutForFirstButShoot.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_LEFT.plus(new Translation2d(0, 1.0)), null, new Rotation2d()),
            new Waypoint(FieldConfig.getInstance().NOTE_H.plus(new Translation2d(Units.feetToMeters(-0.5), -0.1)), new Rotation2d(), null)
        );

        grabFirstPaths.put(CENTERLINE_NOTES.G, grabGFirst);
        grabFirstPaths.put(CENTERLINE_NOTES.F, grabFFirst);
        grabFirstPaths.put(CENTERLINE_NOTES.H, grabHFirst);

        var comeBackScoreG = SwervePathBuilder.generate(4.0, 3.5,
            Waypoint.fromHolonomicPose(grabGFirst.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_LEFT.plus(new Translation2d(0, 1.0)), null, new Rotation2d()),
            Waypoint.fromHolonomicPose(secondShotPose)
        );

        var comeBackScoreF = SwervePathBuilder.generate(4.0, 3.5,
            Waypoint.fromHolonomicPose(grabFFirst.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_LEFT.plus(new Translation2d(0.5, 1.0)), null, new Rotation2d()),
            Waypoint.fromHolonomicPose(secondShotPose.transformBy(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(0))))
        );

        var comeBackScoreH = SwervePathBuilder.generate(4.0, 3.5,
            Waypoint.fromHolonomicPose(grabHFirst.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_LEFT.plus(new Translation2d(0, 1.0)), null, new Rotation2d()),
            Waypoint.fromHolonomicPose(secondShotPose.transformBy(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(5))))
        );

        scorePaths.put(CENTERLINE_NOTES.G, comeBackScoreG);
        scorePaths.put(CENTERLINE_NOTES.F, comeBackScoreF);
        scorePaths.put(CENTERLINE_NOTES.H, comeBackScoreH);

        var grabGSecond = SwervePathBuilder.generate(4.0, 3.0, 0.675, 0.0,
            Waypoint.fromHolonomicPose(secondShotPose),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_LEFT.plus(new Translation2d(0, 1.0)), null, new Rotation2d()),
            new Waypoint(FieldConfig.getInstance().NOTE_G.plus(new Translation2d(Units.feetToMeters(-1), -Units.feetToMeters(0.6))), new Rotation2d(), null)
        );

        var grabFSecond = SwervePathBuilder.generate(4.0, 3.5,
            Waypoint.fromHolonomicPose(secondShotPose),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_LEFT.plus(new Translation2d(0.5, 1.0)), null, new Rotation2d()),
            new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(Units.feetToMeters(-3.0), Units.feetToMeters(-0.5))), new Rotation2d(), new Rotation2d()),
            new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(Units.feetToMeters(0.5), Units.feetToMeters(-0.5))), new Rotation2d(), new Rotation2d())
        );

        var grabHSecond = SwervePathBuilder.generate(4.0, 3.0, 0.675, 0.0,
            Waypoint.fromHolonomicPose(secondShotPose),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_LEFT.plus(new Translation2d(0, 1.0)), null, new Rotation2d()),
            new Waypoint(FieldConfig.getInstance().NOTE_H.plus(new Translation2d(Units.feetToMeters(-1), -Units.feetToMeters(1.5))))
        );

        grabSecondPaths.put(CENTERLINE_NOTES.G, grabGSecond);
        grabSecondPaths.put(CENTERLINE_NOTES.F, grabFSecond);
        grabSecondPaths.put(CENTERLINE_NOTES.H, grabHSecond);

        // comeBackScoreF = SwervePathBuilder.generate(4.0, 3.5,
        //     Waypoint.fromHolonomicPose(goOutForF.getFinalHolonomicPose()),
        //     new Waypoint(FieldConfig.getInstance().BLUE_STAGE_LEFT.plus(new Translation2d(-0.5, 1.0)), null, new Rotation2d(Math.PI)),
        //     Waypoint.fromHolonomicPose(secondShotPose)
        // );

        Rotation2d gobbleKDirection = FieldConfig.getInstance().NOTE_K.minus(secondShotPos).getAngle();

        gobbleK = SwervePathBuilder.generate(4.0, 3.5,
            Waypoint.fromHolonomicPose(comeBackScoreF.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().NOTE_K.minus(new Translation2d(0.5, gobbleKDirection)), gobbleKDirection, gobbleKDirection),
            new Waypoint(FieldConfig.getInstance().NOTE_K.plus(new Translation2d(0.15, 0.15)), gobbleKDirection, gobbleKDirection)
        );

        scoreK = SwervePathBuilder.generate(4.0, 3.5,
            Waypoint.fromHolonomicPose(gobbleK.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().NOTE_K.plus(new Translation2d(-1.0, 0)))
        );

        finalHint = ShotPlanner.getInstance().getRawDistanceToGoal(scoreK.getFinalHolonomicPose().getTranslation());

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
                    drive(goOutForFirstButShoot, () -> RobotContainer.drive.getPose().getTranslation().getX() <= FieldConfig.getInstance().NOTE_J.getX() + 0.5 && RobotContainer.feeder.noteInEntrySensor()),
                    drive(grabFirstPaths.get(this.note1))
                ),
                Commands.waitUntil(() -> RobotContainer.drive.getPose().getTranslation().getX() >= firstShotPos.getX() + Units.feetToMeters(-0.5))
                    .deadlineWith(RobotContainer.hintAng(0, 0))
                    .andThen(
                        RobotContainer.shootSequence(),
                        RobotContainer.intake().alongWith(RobotContainer.wrist.intakeAssist())
                    )
            ),
            RobotContainer.hintAng(4, 2),
            driveAndIntake(scorePaths.get(this.note1), () -> true)
                .deadlineWith(
                    RobotContainer.hintAng(0.5, 0),
                    Commands.print("RUNNING BACK PATH").repeatedly()
                ),
            RobotContainer.shootSequence(),
            driveAndIntake(grabSecondPaths.get(this.note2)).deadlineWith(RobotContainer.wrist.intakeAssist()),
            driveAndIntake(scorePaths.get(this.note2), () -> true)
                .deadlineWith(RobotContainer.hintAng(1.0, 0)),
            RobotContainer.shootSequence(),
            driveAndIntake(gobbleK).deadlineWith(RobotContainer.hintAng(0, 5)),
            driveAndIntake(scoreK, () -> true),
            Commands.waitSeconds(0.25),
            RobotContainer.shootSequence()
        );
    }

    @Override
    public Route getRoute() {
        return new Route()
            .addPaths(goOutForFirstButShoot, grabFirstPaths.get(this.note1), scorePaths.get(this.note1), grabSecondPaths.get(this.note2), scorePaths.get(this.note2), gobbleK, scoreK);
    }
    
}
