package org.frogforce503.robot2024.auto.comp.WorkingAutons.red;

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

public class RedCleanup5OFSZ extends AutoMode {

    PlannedPath grabA, grabB, grabC, grabF;
    PlannedPath scoreF;

    private double maxVel = 4.0;
    private double maxAcc = 3.0;

    private double maxVel_F = 4.6;
    private double maxAcc_F = 4.75;

    private double noteOffset = 0.45;

    private Pair<Double, Double> HINT_NOTE_A = Pair.of(0.0, 5.5);
    private Pair<Double, Double> HINT_NOTE_B = Pair.of(0.0, 6.0);
    private Pair<Double, Double> HINT_NOTE_C = Pair.of(0.0, 5.5);
    private Pair<Double, Double> HINT_NOTE_F = Pair.of(0.0, 1.0);

    public RedCleanup5OFSZ() {

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

        grabA = SwervePathBuilder.generate(maxVel, maxAcc, 
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().NOTE_A.minus(new Translation2d(noteOffset, AToSpeaker.getAngle())), null, AToSpeaker.getAngle())
        );

        grabB = SwervePathBuilder.generate(maxVel, maxAcc, 
            Waypoint.fromHolonomicPose(grabA.getFinalHolonomicPose()).withHolonomicRotation(BToSpeaker.getAngle()),
            new Waypoint(FieldConfig.getInstance().NOTE_A.interpolate(FieldConfig.getInstance().NOTE_B, 0.5).plus(new Translation2d(1.25, 0)), null, BToSpeaker.getAngle()),
            new Waypoint(FieldConfig.getInstance().NOTE_B.minus(new Translation2d(noteOffset, BToSpeaker.getAngle())), null, BToSpeaker.getAngle())
        );

        grabC = SwervePathBuilder.generate(maxVel, maxAcc, 
            Waypoint.fromHolonomicPose(grabB.getFinalHolonomicPose()).withHolonomicRotation(CToSpeaker.getAngle()),
            new Waypoint(FieldConfig.getInstance().NOTE_B.interpolate(FieldConfig.getInstance().NOTE_C, 0.5).plus(new Translation2d(1.25, 0)), null, CToSpeaker.getAngle()),
            new Waypoint(FieldConfig.getInstance().NOTE_C.minus(new Translation2d(noteOffset, CToSpeaker.getAngle())), null, CToSpeaker.getAngle())
        );

        grabF = SwervePathBuilder.generate(maxVel_F, maxAcc_F,
            Waypoint.fromHolonomicPose(grabC.getFinalHolonomicPose()).withHolonomicRotation(new Rotation2d(Math.PI).plus(Rotation2d.fromDegrees(-10))),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_LEFT.interpolate(FieldConfig.getInstance().RED_STAGE_RIGHT, 0.5).interpolate(FieldConfig.getInstance().RED_CENTER_STAGE, 0.5), null, new Rotation2d(Math.PI).plus(Rotation2d.fromDegrees(-10))),
            new Waypoint(FieldConfig.getInstance().NOTE_F, null, new Rotation2d(Math.PI).plus(Rotation2d.fromDegrees(-10)))
        );

        Pose2d shotPose = Waypoints.facingGoal(
            FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(1.5, -0.5))
        );

        scoreF = SwervePathBuilder.generate(maxVel_F, maxAcc_F,
            Waypoint.fromHolonomicPose(grabF.getFinalHolonomicPose()).withHolonomicRotation(shotPose.getRotation()),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_LEFT.interpolate(FieldConfig.getInstance().RED_STAGE_RIGHT, 0.5), null, shotPose.getRotation()),
            Waypoint.fromHolonomicPose(shotPose)
        );
    }

    @Override
    public Command routine() {
        return Commands.sequence(
            RobotContainer.selectPreset(Presets.BATTER),
            RobotContainer.shootSequence(),
            
            driveAndFetchFixed(grabA, 0.75).alongWith(RobotContainer.hintAuton(HINT_NOTE_A)),
            RobotContainer.autonAdjust(),
            RobotContainer.shootSequence().beforeStarting(RobotContainer.hintAuton(HINT_NOTE_A, 0.05)),

            driveAndFetchFixed(grabB, 0.75).alongWith(RobotContainer.hintAuton(HINT_NOTE_B)),
            RobotContainer.autonAdjust(),
            RobotContainer.shootSequence().beforeStarting(RobotContainer.hintAuton(HINT_NOTE_B, 0.05)),

            driveAndFetchFixed(grabC, 0.75).alongWith(RobotContainer.hintAuton(HINT_NOTE_C)),
            RobotContainer.autonAdjust(),
            RobotContainer.shootSequence().beforeStarting(RobotContainer.hintAuton(HINT_NOTE_C, 0.05)),

            driveAndFetchFixed(grabF, 0.75).alongWith(RobotContainer.hintAuton(HINT_NOTE_F)),
            driveAndIntake(scoreF).alongWith(RobotContainer.hintAuton(HINT_NOTE_F)),
            RobotContainer.shootSequence().beforeStarting(RobotContainer.hintAuton(HINT_NOTE_F, 0.05))
        ); 
    }

    @Override
    public Route getRoute() {
        return new Route(grabA, grabB, grabC, grabF, scoreF);
    }
    
}