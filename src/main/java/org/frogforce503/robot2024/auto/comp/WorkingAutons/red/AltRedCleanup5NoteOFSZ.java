/*
 * Shoots preload, picks up and shoot 3 near notes then pick up Note F (center ceterline note)
 */


package org.frogforce503.robot2024.auto.comp.WorkingAutons.red;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

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

public class AltRedCleanup5NoteOFSZ extends AutoMode {

    PlannedPath grabB, grabF, scoreF, grabA, grabC, goForMore;

    private double maxVel = 4.0;
    private double maxAcc = 3.0;

    private double maxVel_F = 4.6;
    private double maxAcc_F = 4.75;

    private double maxVel_MORE = 4.8;
    private double maxAcc_MORE = 4.75;

    private double noteOffset = 0.45;

    private Pair<Double, Double> HINT_NOTE_B = Pair.of(0.0, 5.5);
    private Pair<Double, Double> HINT_NOTE_F = Pair.of(0.0, 5.5);
    private Pair<Double, Double> HINT_NOTE_A = Pair.of(0.0, 5.5);
    private Pair<Double, Double> HINT_NOTE_C = Pair.of(0.0, 5.5);

    public AltRedCleanup5NoteOFSZ() {

        Pose2d startingPose = setupPose(
            Waypoints.facingGoal(new Pose2d(
                FieldConfig.getInstance().RED_INITIATION_LINE + Robot.bot.fullRobotLength / 2,
                FieldConfig.getInstance().RED_SPEAKER.getY(),
                new Rotation2d())
            )
        );

        Translation2d AToSpeaker = FieldConfig.getInstance().NOTE_A.minus(FieldConfig.getInstance().RED_SPEAKER);
        Translation2d BToSpeaker = FieldConfig.getInstance().NOTE_B.minus(FieldConfig.getInstance().RED_SPEAKER);
        Translation2d CToSpeaker = FieldConfig.getInstance().NOTE_C.minus(FieldConfig.getInstance().RED_SPEAKER);

        grabB = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().NOTE_B, null, new Rotation2d(Math.PI))
        );

        grabF = SwervePathBuilder.generate(maxVel_F, maxAcc_F,
            Waypoint.fromHolonomicPose(grabB.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(3.0, 0.0)), null, new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().NOTE_F, null, new Rotation2d(Math.PI))
        );

        // Pose2d shootingPose = Waypoints.facingGoal(
        //     FieldConfig.getInstance().RED_SPEAKER.plus(new Translation2d(-3.0, 0.0))
        // );

        // scoreF = SwervePathBuilder.generate(maxVel_F, maxAcc_F,
        //     Waypoint.fromHolonomicPose(grabF.getFinalHolonomicPose()),
        //     new Waypoint(FieldConfig.getInstance().RED_STAGE_LEFT.interpolate(FieldConfig.getInstance().RED_STAGE_RIGHT, 0.5).interpolate(FieldConfig.getInstance().RED_CENTER_STAGE, 0.5), null, shootingPose.getRotation()),
        //     Waypoint.fromHolonomicPose(shootingPose)
        // );

        scoreF = SwervePathBuilder.reversedOf(grabF, maxVel_F, maxAcc_F);

        grabA = SwervePathBuilder.generate(maxVel, maxAcc, 
            Waypoint.fromHolonomicPose(scoreF.getFinalHolonomicPose()).withHolonomicRotation(AToSpeaker.getAngle()),
            new Waypoint(FieldConfig.getInstance().NOTE_A.interpolate(FieldConfig.getInstance().NOTE_B, 0.5).plus(new Translation2d(0.5, 0.25)), null, AToSpeaker.getAngle()),
            new Waypoint(FieldConfig.getInstance().NOTE_A.minus(new Translation2d(noteOffset, AToSpeaker.getAngle())), null, AToSpeaker.getAngle())
        );

        grabC = SwervePathBuilder.generate(maxVel, maxAcc, 
            Waypoint.fromHolonomicPose(grabA.getFinalHolonomicPose()).withHolonomicRotation(CToSpeaker.getAngle()),
            new Waypoint(FieldConfig.getInstance().NOTE_A.interpolate(FieldConfig.getInstance().NOTE_C, 0.5).plus(new Translation2d(0.75, 0)), null, CToSpeaker.getAngle()),
            new Waypoint(FieldConfig.getInstance().NOTE_C.minus(new Translation2d(noteOffset, CToSpeaker.getAngle())), null, CToSpeaker.getAngle())
        );
        
        goForMore = SwervePathBuilder.generate(maxVel_MORE, maxAcc_MORE, 
            Waypoint.fromHolonomicPose(grabC.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(-2.25, 1.25)), null, new Rotation2d(Math.PI))
        );
    }

    @Override
    public Command routine() {
        return Commands.sequence(
            RobotContainer.selectPreset(Presets.BATTER),
            RobotContainer.shootSequence(),

            driveAndIntake(grabB).deadlineWith(RobotContainer.hintAuton(HINT_NOTE_B)),
            RobotContainer.shootSequence().beforeStarting(RobotContainer.hintAuton(HINT_NOTE_B, 0.05)),

            driveAndFetchFixed(grabF, 0.75).deadlineWith(RobotContainer.hintAuton(HINT_NOTE_F)),
            driveAndIntake(scoreF).alongWith(RobotContainer.hintAuton(HINT_NOTE_F)).withTimeout(scoreF.getTotalTimeSeconds()),
            RobotContainer.shootSequence().beforeStarting(RobotContainer.hintAuton(HINT_NOTE_F, 0.05)),

            driveAndFetchFixed(grabA, 0.25).deadlineWith(RobotContainer.hintAuton(HINT_NOTE_A)),
            RobotContainer.autonAdjust(),
            RobotContainer.shootSequence().beforeStarting(RobotContainer.hintAuton(HINT_NOTE_A, 0.05)),

            driveAndFetchFixed(grabC, 0.25).deadlineWith(RobotContainer.hintAuton(HINT_NOTE_C)),
            RobotContainer.autonAdjust(),
            RobotContainer.shootSequence().beforeStarting(RobotContainer.hintAuton(HINT_NOTE_C, 0.05)),

            driveAndFetchFixed(goForMore, 0.75)
        );
    }

    @Override
    public Route getRoute() {
        return new Route(grabB, grabF, scoreF, grabA, grabC, goForMore);
    }
    
}