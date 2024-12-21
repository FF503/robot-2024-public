package org.frogforce503.robot2024.auto.comp.districts.blue;

import static org.frogforce503.robot2024.auto.AutoUtil.Waypoints.rotation;
import static org.frogforce503.robot2024.auto.AutoUtil.Waypoints.translation2d;

import org.frogforce503.lib.auto.AutoMode;
import org.frogforce503.lib.trajectory.PlannedPath;
import org.frogforce503.lib.trajectory.Route;
import org.frogforce503.lib.trajectory.SwervePathBuilder;
import org.frogforce503.robot2024.Robot;
import org.frogforce503.robot2024.RobotContainer;
import org.frogforce503.robot2024.auto.AutoUtil.Waypoints;
import org.frogforce503.robot2024.fields.FieldConfig;
import org.frogforce503.robot2024.planners.ShotPlanner;
import org.frogforce503.lib.trajectory.Waypoint;
import org.frogforce503.robot2024.auto.AutoUtil.Waypoints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;


// Using 3 near notes
public class BlueCenterCleanup5 extends AutoMode {

    PlannedPath goOutForG1, goOutForG2, comeBackScoreG, grabI, grabJ, grabK, goHome;
    private double maxSpeed = 4;
    private double maxAccel = 3;

    private double hintDistance = 0.0;

    private final double steadyShotDistance = 3.711; // TODO: tune based on the value in the shotmap with 0 wrist 0 arm
    Translation2d firstShotPos;

    Translation2d secondShotPos;
    double nextShotsHint, finalHint;

    public BlueCenterCleanup5() {
        // Pose2d startingPose = setupPose(
        //     Waypoints.facingGoal(new Pose2d(
        //         FieldConfig.getInstance().INITIATION_LINE_X() - Robot.bot.fullRobotLength / 2,
        //         FieldConfig.getInstance().NOTE_CENTER().getY(),
        //         new Rotation2d())
        //     )
        // );

        Pose2d startingPose = setupPose(
            Waypoints.facingGoal(FieldConfig.getInstance().NOTE_K.interpolate(FieldConfig.getInstance().NOTE_J, 0.5).plus(new Translation2d(-1.0, 0.0)))
        );

        firstShotPos = FieldConfig.getInstance().BLUE_SPEAKER.plus(new Translation2d(steadyShotDistance, 0.75));

        Pose2d firstShotPose = Waypoints.facingGoal(firstShotPos);

        secondShotPos = FieldConfig.getInstance().BLUE_SPEAKER.plus(new Translation2d(1.75, 0));
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
            new Waypoint(FieldConfig.getInstance().NOTE_G.plus(new Translation2d(0, 0)))
        );

        comeBackScoreG = SwervePathBuilder.generate(4.0, 3.5,
            Waypoint.fromHolonomicPose(goOutForG2.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_LEFT.plus(new Translation2d(0, 1.0)), null, new Rotation2d()),
            Waypoint.fromHolonomicPose(secondShotPose)
        );

        Translation2d IToSpeaker = FieldConfig.getInstance().NOTE_SOURCE_SIDE().minus(FieldConfig.getInstance().SPEAKER_AUTON());
        Translation2d JToSpeaker = FieldConfig.getInstance().NOTE_CENTER().minus(FieldConfig.getInstance().SPEAKER_AUTON());
        Translation2d KToSpeaker = FieldConfig.getInstance().NOTE_AMP_SIDE().minus(FieldConfig.getInstance().SPEAKER_AUTON());

        grabI = SwervePathBuilder.generate(maxSpeed, maxAccel, 
            Waypoint.fromHolonomicPose(comeBackScoreG.getFinalHolonomicPose()).withDriveRotation(new Rotation2d()),
            new Waypoint(FieldConfig.getInstance().NOTE_SOURCE_SIDE().minus(new Translation2d(Units.feetToMeters(1.25), IToSpeaker.getAngle())), IToSpeaker.getAngle(), IToSpeaker.getAngle())
        );

        hintDistance = ShotPlanner.getInstance().getRawDistanceToGoal(grabI.getFinalHolonomicPose().getTranslation());

        grabJ = SwervePathBuilder.generate(maxSpeed, maxAccel, 
            Waypoint.fromHolonomicPose(grabI.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().NOTE_SOURCE_SIDE().interpolate(FieldConfig.getInstance().NOTE_CENTER(), 0.5).minus(new Translation2d(1.25, 0))),
            new Waypoint(FieldConfig.getInstance().NOTE_CENTER().minus(new Translation2d(Units.feetToMeters(1.25), JToSpeaker.getAngle())), JToSpeaker.getAngle(), JToSpeaker.getAngle())
        );

        grabK = SwervePathBuilder.generate(maxSpeed, maxAccel, 
            Waypoint.fromHolonomicPose(grabJ.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().NOTE_AMP_SIDE().interpolate(FieldConfig.getInstance().NOTE_CENTER(), 0.5).minus(new Translation2d(1.25, 0))),
            new Waypoint(FieldConfig.getInstance().NOTE_AMP_SIDE().minus(new Translation2d(Units.feetToMeters(1.25), KToSpeaker.getAngle())), KToSpeaker.getAngle(), KToSpeaker.getAngle())
        );

        goHome = SwervePathBuilder.generate(maxSpeed, maxAccel, 
            Waypoint.fromHolonomicPose(grabK.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().NOTE_AMP_SIDE().minus(new Translation2d(1.75, 0)), null, new Rotation2d())
        );
    }

    @Override
    public Command routine() {
        // TODO Auto-generated method stub
        return Commands.sequence(
            Commands.deadline( // grabs G while shooting P
                Commands.sequence(
                    drive(goOutForG1, () -> RobotContainer.drive.getPose().getTranslation().getX() >= FieldConfig.getInstance().NOTE_B.getX() - 0.5 && RobotContainer.feeder.noteInEntrySensor()),
                    drive(goOutForG2, 0.75)
                ),
                Commands.waitUntil(() -> RobotContainer.drive.getPose().getTranslation().getX() >= firstShotPos.getX() - Units.feetToMeters(1.0))
                    .deadlineWith(RobotContainer.hint(steadyShotDistance + Units.feetToMeters(0.875)))
                    .andThen(
                        RobotContainer.shootSequence(),
                        RobotContainer.intake().alongWith(RobotContainer.wrist.intakeAssist())
                    )
            ),
            driveAndIntake(comeBackScoreG, () -> true)
                .deadlineWith(RobotContainer.hint(nextShotsHint)),
            RobotContainer.shootSequence(),
            driveAndIntakeParallel(grabI, () -> true).withTimeout(3).deadlineWith(RobotContainer.hint(hintDistance)), 
            Commands.waitSeconds(0.25),
            RobotContainer.shootSequence(),
            driveAndIntakeParallel(grabJ, () -> true).withTimeout(3),
            Commands.waitSeconds(0.25),
            RobotContainer.shootSequence(),
            driveAndIntakeParallel(grabK, () -> true).withTimeout(3),
            Commands.waitSeconds(0.25),
            RobotContainer.shootSequence()
        );
    }

    @Override
    public Route getRoute() {
        // TODO Auto-generated method stub
    return new Route(goOutForG1, goOutForG2, comeBackScoreG, grabI, grabJ, grabK, goHome);
    }
    
}
