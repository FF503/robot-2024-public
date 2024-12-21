package org.frogforce503.robot2024.auto.comp.WorkingAutons.blue;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import org.frogforce503.lib.auto.AutoChooser.CENTERLINE_NOTES;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AltBlueCleanup5NoteOFSZ extends AutoMode{
    PlannedPath grabJ, grabF, scoreF, grabI, grabK, goForMore;

    private double maxVel = 4.0;
    private double maxAcc = 3.0;

    private double maxVel_F = 4.6;
    private double maxAcc_F = 4.75;

    private double maxVel_MORE = 4.8;
    private double maxAcc_MORE = 4.75;

    private double noteOffset = 0.45;

    private Pair<Double, Double> HINT_NOTE_J = Pair.of(0.0, 5.5);
    private Pair<Double, Double> HINT_NOTE_F = Pair.of(0.0, 5.5);
    private Pair<Double, Double> HINT_NOTE_I = Pair.of(0.0, 5.5);
    private Pair<Double, Double> HINT_NOTE_K = Pair.of(0.0, 5.5);

    public AltBlueCleanup5NoteOFSZ() {

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

        grabJ = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().NOTE_J, null, new Rotation2d())
        );

        grabF = SwervePathBuilder.generate(maxVel_F, maxAcc_F,
            Waypoint.fromHolonomicPose(grabJ.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(-3.0, 0.0)), null, new Rotation2d()),
            new Waypoint(FieldConfig.getInstance().NOTE_F, null, new Rotation2d())
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

        grabI = SwervePathBuilder.generate(maxVel, maxAcc, 
            Waypoint.fromHolonomicPose(scoreF.getFinalHolonomicPose()).withHolonomicRotation(IToSpeaker.getAngle()),
            new Waypoint(FieldConfig.getInstance().NOTE_I.interpolate(FieldConfig.getInstance().NOTE_J, 0.5).plus(new Translation2d(-0.5, 0.25)), null, IToSpeaker.getAngle()),
            new Waypoint(FieldConfig.getInstance().NOTE_I.minus(new Translation2d(noteOffset, IToSpeaker.getAngle())), null, IToSpeaker.getAngle())
        );

        grabK = SwervePathBuilder.generate(maxVel, maxAcc, 
            Waypoint.fromHolonomicPose(grabI.getFinalHolonomicPose()).withHolonomicRotation(KToSpeaker.getAngle()),
            new Waypoint(FieldConfig.getInstance().NOTE_I.interpolate(FieldConfig.getInstance().NOTE_K, 0.5).plus(new Translation2d(-0.75, 0)), null, KToSpeaker.getAngle()),
            new Waypoint(FieldConfig.getInstance().NOTE_K.minus(new Translation2d(noteOffset, KToSpeaker.getAngle())), null, KToSpeaker.getAngle())
        );
        
        goForMore = SwervePathBuilder.generate(maxVel_MORE, maxAcc_MORE, 
            Waypoint.fromHolonomicPose(grabK.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_LEFT.plus(new Translation2d(2.25, 1.25)), null, new Rotation2d())
        );
    }

    @Override
    public Command routine() {
        return Commands.sequence(
            RobotContainer.selectPreset(Presets.BATTER),
            RobotContainer.shootSequence(),

            driveAndIntake(grabJ).deadlineWith(RobotContainer.hintAuton(HINT_NOTE_J)),
            RobotContainer.shootSequence().beforeStarting(RobotContainer.hintAuton(HINT_NOTE_J, 0.05)),

            driveAndFetchFixed(grabF, 0.75).deadlineWith(RobotContainer.hintAuton(HINT_NOTE_F)),
            driveAndIntake(scoreF).alongWith(RobotContainer.hintAuton(HINT_NOTE_F)).withTimeout(scoreF.getTotalTimeSeconds()),
            RobotContainer.shootSequence().beforeStarting(RobotContainer.hintAuton(HINT_NOTE_F, 0.05)),

            driveAndFetchFixed(grabI, 0.25).deadlineWith(RobotContainer.hintAuton(HINT_NOTE_I)),
            RobotContainer.autonAdjust(),
            RobotContainer.shootSequence().beforeStarting(RobotContainer.hintAuton(HINT_NOTE_I, 0.05)),

            driveAndFetchFixed(grabK, 0.25).deadlineWith(RobotContainer.hintAuton(HINT_NOTE_K)),
            RobotContainer.autonAdjust(),
            RobotContainer.shootSequence().beforeStarting(RobotContainer.hintAuton(HINT_NOTE_K, 0.05)),

            driveAndFetchFixed(goForMore, 0.75)
        );
    }

    @Override
    public Route getRoute() {
        return new Route(grabJ, grabF, scoreF, grabI, grabK, goForMore);
    }
    
}