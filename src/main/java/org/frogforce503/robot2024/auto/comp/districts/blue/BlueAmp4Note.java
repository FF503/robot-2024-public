package org.frogforce503.robot2024.auto.comp.districts.blue;

// import static edu.wpi.first.units.Units.Rotation;

import org.frogforce503.lib.auto.AutoMode;
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
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.EllipticalRegionConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class BlueAmp4Note extends AutoMode {

    private PlannedPath goOutForG1, goOutForG2, comeBackScoreG, goOutForF, comeBackScoreF, gobbleK, scoreK;

    private final double steadyShotDistance = 3.711; // TODO: tune based on the value in the shotmap with 0 wrist 0 arm
    Translation2d firstShotPos;

    Translation2d secondShotPos;

    double nextShotsHint, finalHint;

    public BlueAmp4Note() {

        Pose2d startingPose = setupPose(
            Waypoints.facingGoal(FieldConfig.getInstance().NOTE_K.interpolate(FieldConfig.getInstance().NOTE_J, 0.5).plus(new Translation2d(-1.0, 0.0)))
        );

        firstShotPos = FieldConfig.getInstance().BLUE_SPEAKER.plus(new Translation2d(steadyShotDistance, 0.75));

        Pose2d firstShotPose = Waypoints.facingGoal(firstShotPos);

        secondShotPos = firstShotPos.plus(new Translation2d(0.5, 0));
        nextShotsHint = ShotPlanner.getInstance().getRawDistanceToGoal(secondShotPos) - 0.25;

        Pose2d secondShotPose = Waypoints.facingGoal(secondShotPos);

        // TrajectoryConstraint slowDownWhenShooting = new RectangularRegionConstraint(
        //     firstShotPos.plus(new Translation2d(-0.5, -0.5)), 
        //     firstShotPos.plus(new Translation2d(0.5, 0.5)), 
        //     new MaxVelocityConstraint(0.25)
        // );
        
        goOutForG1 = SwervePathBuilder.generate(4.0, 3.0, 0.0, 0.675,
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().NOTE_K.interpolate(FieldConfig.getInstance().NOTE_J, 0.5), new Rotation2d(), firstShotPose.getRotation()),
            Waypoint.fromHolonomicPose(firstShotPose)
        );

        goOutForG2 = SwervePathBuilder.generate(4.0, 3.0, 0.675, 0.0,
            Waypoint.fromHolonomicPose(goOutForG1.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_LEFT.plus(new Translation2d(0, 1.0)), null, new Rotation2d()),
            new Waypoint(FieldConfig.getInstance().NOTE_G.plus(new Translation2d(Units.feetToMeters(2), 0)))
        );

        comeBackScoreG = SwervePathBuilder.generate(4.0, 3.5,
            Waypoint.fromHolonomicPose(goOutForG2.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_LEFT.plus(new Translation2d(0, 1.0)), null, new Rotation2d()),
            Waypoint.fromHolonomicPose(secondShotPose)
        );

        goOutForF = SwervePathBuilder.generate(4.0, 3.5,
            Waypoint.fromHolonomicPose(comeBackScoreG.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_LEFT.plus(new Translation2d(0.5, 1.0)), null, new Rotation2d()),
            new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(Units.feetToMeters(2.5), 0)), new Rotation2d(), new Rotation2d()),
            new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(Units.feetToMeters(0.5), 0)), new Rotation2d(), new Rotation2d())
        );

        comeBackScoreF = SwervePathBuilder.generate(4.0, 3.5,
            Waypoint.fromHolonomicPose(goOutForF.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_LEFT.plus(new Translation2d(0.5, 1.0)), null, new Rotation2d()),
            Waypoint.fromHolonomicPose(secondShotPose)
        );

        Rotation2d gobbleCDirection = FieldConfig.getInstance().NOTE_K.minus(secondShotPos).getAngle();

        gobbleK = SwervePathBuilder.generate(4.0, 3.5,
            Waypoint.fromHolonomicPose(comeBackScoreF.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().NOTE_K.minus(new Translation2d(0.5, gobbleCDirection)), gobbleCDirection, gobbleCDirection),
            new Waypoint(FieldConfig.getInstance().NOTE_K, gobbleCDirection, gobbleCDirection)
        );

        scoreK = SwervePathBuilder.generate(4.0, 3.5,
            Waypoint.fromHolonomicPose(gobbleK.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().NOTE_K.plus(new Translation2d(-0.75, 0)))
        );

        finalHint = ShotPlanner.getInstance().getRawDistanceToGoal(scoreK.getFinalHolonomicPose().getTranslation());
    }

    @Override
    public Command routine() {
        return Commands.sequence(
            Commands.deadline( // grabs G while shooting P
                Commands.sequence(
                    drive(goOutForG1, () -> RobotContainer.drive.getPose().getTranslation().getX() >= FieldConfig.getInstance().NOTE_B.getX() - 0.5 && RobotContainer.feeder.noteInEntrySensor()),
                    drive(goOutForG2, 0.75)
                ),
                Commands.waitUntil(() -> RobotContainer.drive.getPose().getTranslation().getX() >= firstShotPos.getX())
                    .deadlineWith(RobotContainer.hint(steadyShotDistance + Units.feetToMeters(0.875)))
                    .andThen(
                        RobotContainer.shootSequence(),
                        RobotContainer.intake().alongWith(RobotContainer.wrist.intakeAssist())
                    )
            ),
            driveAndIntake(comeBackScoreG, () -> true)
                .deadlineWith(RobotContainer.hint(nextShotsHint)),
            RobotContainer.shootSequence(),
            driveAndIntake(goOutForF).deadlineWith(RobotContainer.wrist.intakeAssist()),
            driveAndIntake(comeBackScoreF, () -> true)
                .deadlineWith(RobotContainer.hint(nextShotsHint + 0.15)),
            RobotContainer.shootSequence(),
            driveAndIntake(gobbleK).deadlineWith(RobotContainer.hint(finalHint)),
            driveAndIntake(scoreK, () -> true),
            RobotContainer.shootSequence()
        );
    }

    @Override
    public Route getRoute() {
        return new Route(goOutForG1, goOutForG2, comeBackScoreG, goOutForF, comeBackScoreF, gobbleK, scoreK);
    }
    
}
