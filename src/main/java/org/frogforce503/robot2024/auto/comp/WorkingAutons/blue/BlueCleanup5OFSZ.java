package org.frogforce503.robot2024.auto.comp.WorkingAutons.blue;


import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import org.frogforce503.lib.auto.AutoMode;
import org.frogforce503.lib.trajectory.PlannedPath;
import org.frogforce503.lib.trajectory.Route;
import org.frogforce503.lib.trajectory.SwervePathBuilder;
import org.frogforce503.robot2024.Robot;
import org.frogforce503.robot2024.RobotContainer;
import org.frogforce503.robot2024.auto.AutoUtil.Waypoints;
import org.frogforce503.robot2024.fields.FieldConfig;
import org.frogforce503.robot2024.planners.ShotPlanner.Presets;
import org.frogforce503.lib.trajectory.Waypoint;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class BlueCleanup5OFSZ extends AutoMode {

    PlannedPath grabI, grabJ, grabK, grabF;
    PlannedPath scoreF;
    
    private double maxVel = 4.0;
    private double maxAcc = 3.0;

    private double maxVel_F = 4.6;
    private double maxAcc_F = 4.75;

    private double noteOffset = 0.45;

    private Pair<Double, Double> HINT_NOTE_I = Pair.of(0.0, 5.5);
    private Pair<Double, Double> HINT_NOTE_J = Pair.of(0.0, 6.0);
    private Pair<Double, Double> HINT_NOTE_K = Pair.of(0.0, 5.5);
    private Pair<Double, Double> HINT_NOTE_F = Pair.of(0.0, 1.0);

    public BlueCleanup5OFSZ() {

        Pose2d startingPose = setupPose(
            Waypoints.facingGoal(new Pose2d(
                FieldConfig.getInstance().BLUE_INITIATION_LINE - Robot.bot.fullRobotLength / 2,
                FieldConfig.getInstance().BLUE_SPEAKER.getY(),
                new Rotation2d())
            )
        );

        Translation2d IToSpeaker = FieldConfig.getInstance().NOTE_I.minus(FieldConfig.getInstance().BLUE_SPEAKER);
        Translation2d JToSpeaker = FieldConfig.getInstance().NOTE_J.minus(FieldConfig.getInstance().BLUE_SPEAKER);
        Translation2d KToSpeaker = FieldConfig.getInstance().NOTE_K.minus(FieldConfig.getInstance().BLUE_SPEAKER);

        grabI = SwervePathBuilder.generate(maxVel, maxAcc, 
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().NOTE_I.minus(new Translation2d(noteOffset, IToSpeaker.getAngle())), null, IToSpeaker.getAngle())
        );

        grabJ = SwervePathBuilder.generate(maxVel, maxAcc, 
            Waypoint.fromHolonomicPose(grabI.getFinalHolonomicPose()).withHolonomicRotation(JToSpeaker.getAngle()),
            new Waypoint(FieldConfig.getInstance().NOTE_I.interpolate(FieldConfig.getInstance().NOTE_J, 0.5).plus(new Translation2d(-1.25, 0)), null, JToSpeaker.getAngle()),
            new Waypoint(FieldConfig.getInstance().NOTE_J.minus(new Translation2d(noteOffset, JToSpeaker.getAngle())), null, JToSpeaker.getAngle())
        );

        grabK = SwervePathBuilder.generate(maxVel, maxAcc, 
            Waypoint.fromHolonomicPose(grabJ.getFinalHolonomicPose()).withHolonomicRotation(KToSpeaker.getAngle()),
            new Waypoint(FieldConfig.getInstance().NOTE_J.interpolate(FieldConfig.getInstance().NOTE_K, 0.5).plus(new Translation2d(-1.25, 0)), null, KToSpeaker.getAngle()),
            new Waypoint(FieldConfig.getInstance().NOTE_K.minus(new Translation2d(noteOffset, KToSpeaker.getAngle())), null, KToSpeaker.getAngle())
        );

        grabF = SwervePathBuilder.generate(maxVel_F, maxAcc_F,
            Waypoint.fromHolonomicPose(grabK.getFinalHolonomicPose()).withHolonomicRotation(Rotation2d.fromDegrees(10)),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_RIGHT.interpolate(FieldConfig.getInstance().BLUE_STAGE_LEFT, 0.5).interpolate(FieldConfig.getInstance().BLUE_CENTER_STAGE, 0.5), null, Rotation2d.fromDegrees(10)),
            new Waypoint(FieldConfig.getInstance().NOTE_F, null, Rotation2d.fromDegrees(10))
        );

        Pose2d shotPose = Waypoints.facingGoal(
            FieldConfig.getInstance().BLUE_STAGE_LEFT.plus(new Translation2d(-1.5, -0.5))
        );

        scoreF = SwervePathBuilder.generate(maxVel_F, maxAcc_F,
            Waypoint.fromHolonomicPose(grabF.getFinalHolonomicPose()).withHolonomicRotation(shotPose.getRotation()),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_RIGHT.interpolate(FieldConfig.getInstance().BLUE_STAGE_LEFT, 0.5), null, shotPose.getRotation()),
            Waypoint.fromHolonomicPose(shotPose)
        );
    }

    @Override
    public Command routine() {
        return Commands.sequence(
            RobotContainer.selectPreset(Presets.BATTER),
            RobotContainer.shootSequence(),
            
            driveAndIntake(grabI),
            // RobotContainer.shootSequence().beforeStarting(RobotContainer.hintAuton(HINT_NOTE_I, 0.05)),

            driveAndIntake(grabJ),
            // RobotContainer.shootSequence().beforeStarting(RobotContainer.hintAuton(HINT_NOTE_J, 0.05)),

            driveAndIntake(grabK),
            // RobotContainer.shootSequence().beforeStarting(RobotContainer.hintAuton(HINT_NOTE_K, 0.05)),

            driveAndIntake(grabF),
            driveAndIntake(scoreF)
            // RobotContainer.shootSequence().beforeStarting(RobotContainer.hintAuton(HINT_NOTE_F, 0.05))
        );
    }

    @Override
    public Route getRoute() {
        return new Route(grabI, grabJ, grabK, grabF, scoreF);
    }
    
}