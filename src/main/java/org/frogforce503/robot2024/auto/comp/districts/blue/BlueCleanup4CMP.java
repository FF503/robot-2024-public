package org.frogforce503.robot2024.auto.comp.districts.blue;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
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
import org.frogforce503.robot2024.planners.ShotPlanner.Presets;
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
public class BlueCleanup4CMP extends AutoMode {

    PlannedPath grabI, grabJ, grabK, runToF;
    private double maxSpeed = 4;
    private double maxAccel = 3;
    private double noteOffset = 1.25;

    private double hintDistance = 0.0;

    public BlueCleanup4CMP() {
        Pose2d startingPose = setupPose(
            Waypoints.facingGoal(new Pose2d(
                FieldConfig.getInstance().INITIATION_LINE_X() - Robot.bot.fullRobotLength / 2,
                FieldConfig.getInstance().NOTE_SOURCE_SIDE().getY(),
                new Rotation2d())
            )
        );

        Translation2d IToSpeaker = FieldConfig.getInstance().NOTE_SOURCE_SIDE().minus(FieldConfig.getInstance().SPEAKER_AUTON());
        Translation2d JToSpeaker = FieldConfig.getInstance().NOTE_CENTER().minus(FieldConfig.getInstance().SPEAKER_AUTON());
        Translation2d KToSpeaker = FieldConfig.getInstance().NOTE_AMP_SIDE().minus(FieldConfig.getInstance().SPEAKER_AUTON());

        grabI = SwervePathBuilder.generate(maxSpeed, maxAccel, 
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().NOTE_SOURCE_SIDE().minus(new Translation2d(Units.feetToMeters(noteOffset), IToSpeaker.getAngle())), IToSpeaker.getAngle(), IToSpeaker.getAngle().plus(Rotation2d.fromDegrees(5)))
        );

        hintDistance = ShotPlanner.getInstance().getRawDistanceToGoal(grabI.getFinalHolonomicPose().getTranslation());

        grabJ = SwervePathBuilder.generate(maxSpeed, maxAccel, 
            Waypoint.fromHolonomicPose(grabI.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().NOTE_SOURCE_SIDE().interpolate(FieldConfig.getInstance().NOTE_CENTER(), 0.5).minus(new Translation2d(1.25, 0))),
            new Waypoint(FieldConfig.getInstance().NOTE_CENTER().minus(new Translation2d(Units.feetToMeters(noteOffset), JToSpeaker.getAngle())).plus(new Translation2d(0.2, 0)), JToSpeaker.getAngle(), JToSpeaker.getAngle())
        );

        grabK = SwervePathBuilder.generate(maxSpeed, maxAccel, 
            Waypoint.fromHolonomicPose(grabJ.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().NOTE_AMP_SIDE().interpolate(FieldConfig.getInstance().NOTE_CENTER(), 0.5).minus(new Translation2d(1.25, 0))),
            new Waypoint(FieldConfig.getInstance().NOTE_AMP_SIDE().minus(new Translation2d(Units.feetToMeters(noteOffset), KToSpeaker.getAngle())), KToSpeaker.getAngle(), KToSpeaker.getAngle().plus(Rotation2d.fromDegrees(7.5)))
        );

        runToF = SwervePathBuilder.generate(5.5, 5.0, 
            Waypoint.fromHolonomicPose(grabK.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().BLUE_CENTER_STAGE.interpolate(FieldConfig.getInstance().BLUE_STAGE_LEFT, 0.45)),
            new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(-Units.feetToMeters(1.0), 0.0)), new Rotation2d(), new Rotation2d())
        );
    }

    @Override
    public Command routine() {
        // TODO Auto-generated method stub
        return Commands.sequence(
            RobotContainer.selectPreset(Presets.BATTER),
            RobotContainer.shootSequence(),
            RobotContainer.selectPreset(Presets.PODIUM),
            driveAndIntakeParallel(grabI, () -> true).withTimeout(2.5),
            Commands.waitSeconds(0.15),
            RobotContainer.shootSequence(),
            driveAndIntakeParallel(grabJ, () -> true).deadlineWith(RobotContainer.hintAng(0, 8)).withTimeout(2.5),
            Commands.waitSeconds(0.15),
            RobotContainer.shootSequence(),
            driveAndIntakeParallel(grabK, () -> true).deadlineWith(RobotContainer.hintAng(0, 8)).withTimeout(2.5),
            Commands.waitSeconds(0.15),
            RobotContainer.shootSequence(),
            // driveAndIntake(runToF)
            drive(runToF).alongWith(RobotContainer.intake())
        );
    }

    @Override
    public Route getRoute() {
        // TODO Auto-generated method stub
    return new Route(grabI, grabJ, grabK, runToF);
    }
    
}
