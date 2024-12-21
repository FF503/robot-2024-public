package org.frogforce503.robot2024.auto.comp.districts.blue;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import org.frogforce503.lib.auto.AutoMode;
import org.frogforce503.lib.trajectory.PlannedPath;
import org.frogforce503.lib.trajectory.Route;
import org.frogforce503.lib.trajectory.SwervePathBuilder;
import org.frogforce503.lib.trajectory.Waypoint;
import org.frogforce503.lib.trajectory.Route.Tree;
import org.frogforce503.lib.trajectory.Waypoint.Tag;
import org.frogforce503.lib.util.AllianceFlipUtil;
import org.frogforce503.robot2024.Robot;
import org.frogforce503.robot2024.RobotContainer;
import org.frogforce503.robot2024.auto.AutoUtil.Waypoints;
import org.frogforce503.robot2024.fields.FieldConfig;
import org.frogforce503.robot2024.subsystems.NoteDetector.SIDE;
import org.frogforce503.robot2024.subsystems.NoteDetector.SORTING_MODE;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static org.frogforce503.robot2024.auto.AutoUtil.Waypoints.*;

public class BlueAmp3Note extends AutoMode {

    private PlannedPath driveAndShootPreload, gobbleH, scoreH, gobbleG, scoreGAndGetK;
    private Tree grabFirstTree, scoreFirstTree;
    private Tag firstDiversion = new Tag();

    double vMax = 4.0;
    double aMax = 3.0;
    double vWhileMoving = 3.0;

    public BlueAmp3Note() {
        RobotContainer.noteDetector.setSortingMode(SORTING_MODE.LEFT_TO_RIGHT);

        Pose2d startingPose = ((new Pose2d(
            FieldConfig.getInstance().INITIATION_LINE_X() - Units.inchesToMeters(21),
            FieldConfig.getInstance().FIELD_DIMENSIONS.getY() - Units.inchesToMeters(33),
            rotation())
        ));

        Pose2d shootingPosition = /*Waypoints.facingGoal*/(new Pose2d(
            FieldConfig.getInstance().STAGE_AMP_SIDE().plus(translation(-Units.feetToMeters(2.0), Units.feetToMeters(3.5))),
            rotation())
        );

        driveAndShootPreload = SwervePathBuilder.generate(vMax, aMax, 0.0, vWhileMoving,
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().NOTE_AMP_SIDE().plus(translation(0, 0.75)), null, rotation(0)),
            new Waypoint(new Translation2d(FieldConfig.getInstance().WING_LINE_X()-Units.feetToMeters(2), FieldConfig.getInstance().NOTE_H.getY()), rotation(), rotation())
        );

        var grabFirstTrunk = SwervePathBuilder.generate(vMax, aMax, vWhileMoving, vMax*0.8,
            driveAndShootPreload.getFinalWaypoint(),
            new Waypoint(new Translation2d(FieldConfig.getInstance().WING_LINE_X(), FieldConfig.getInstance().NOTE_H.getY()), rotation(), rotation())
        );

        var grabHBranch = SwervePathBuilder.generate(vMax, aMax, vMax*0.8, 0,
            grabFirstTrunk.getFinalWaypoint(),
            new Waypoint(FieldConfig.getInstance().NOTE_H.plus(new Translation2d(-Units.feetToMeters(2.5), 0)), rotation(), rotation())
        );

        var grabGBranch = SwervePathBuilder.generate(vMax, aMax, vMax*0.8, 0,
            grabFirstTrunk.getFinalWaypoint(),
            new Waypoint(FieldConfig.getInstance().NOTE_H.interpolate(FieldConfig.getInstance().NOTE_G, 0.6).plus(new Translation2d(-Units.feetToMeters(4.25), 0))),
            new Waypoint(FieldConfig.getInstance().NOTE_G.plus(new Translation2d(Units.feetToMeters(-1.75), Units.feetToMeters(-0.75))), rotation(), rotation(-35))
        );

        var grabNoneBranch = SwervePathBuilder.generate(vMax, aMax, vMax*0.8, 0,
            grabFirstTrunk.getFinalWaypoint(),
            new Waypoint(new Translation2d(FieldConfig.getInstance().WING_LINE_X() + Units.feetToMeters(1), FieldConfig.getInstance().NOTE_H.getY()), rotation(), rotation())
        );

        grabFirstTree = new Tree()
            .withTrunk(grabFirstTrunk)
            .withBranches(grabHBranch, grabHBranch, grabGBranch, grabNoneBranch); // grabH must be there twice

        var scoreHBranch = SwervePathBuilder.generate(vMax, aMax,
            Waypoint.fromHolonomicPose(grabHBranch.getFinalHolonomicPose()),
            Waypoint.fromHolonomicPose((shootingPosition), rotation(180))
        );

        var scoreGBranch = SwervePathBuilder.generate(vMax, aMax,
            Waypoint.fromHolonomicPose(grabGBranch.getFinalHolonomicPose()),
            Waypoint.fromHolonomicPose((shootingPosition), rotation(180))
        );

        var scoreNoneBranch = SwervePathBuilder.generate(vMax, aMax,
            Waypoint.fromHolonomicPose(grabNoneBranch.getFinalHolonomicPose()),
            Waypoint.fromHolonomicPose((shootingPosition), rotation(180))
        );

        scoreFirstTree = new Tree()
            .withBranches(scoreHBranch, scoreHBranch, scoreGBranch, scoreNoneBranch);

        gobbleH = SwervePathBuilder.generate(vMax, aMax,
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().NOTE_AMP_SIDE().plus(translation(0, 0.75)), null, rotation(0)),
            new Waypoint(FieldConfig.getInstance().NOTE_H.plus(new Translation2d(-Units.feetToMeters(2.5), 0)), rotation(), rotation())
        );

        scoreH = SwervePathBuilder.generate(vMax, aMax,
            Waypoint.fromHolonomicPose(gobbleH.getFinalHolonomicPose()),
            Waypoint.fromHolonomicPose((shootingPosition), rotation(180))
        );

        gobbleG = SwervePathBuilder.generate(vMax, aMax,
            Waypoint.fromHolonomicPose((scoreH.getFinalHolonomicPose())),
            new Waypoint(FieldConfig.getInstance().NOTE_G, rotation(), rotation(-20))
        );

        scoreGAndGetK = SwervePathBuilder.generate(vMax, aMax,
            Waypoint.fromHolonomicPose(gobbleG.getFinalHolonomicPose()),
            new Waypoint(new Translation2d(FieldConfig.getInstance().WING_LINE_X()-Units.feetToMeters(2), FieldConfig.getInstance().NOTE_H.getY()), rotation(180), rotation(0)),
            new Waypoint(FieldConfig.getInstance().NOTE_AMP_SIDE().plus(translation(0, 0.75)), null, rotation(0)),
            new Waypoint(FieldConfig.getInstance().NOTE_AMP_SIDE().plus(translation(-0.5, 0.5)), null, rotation(0)),
            new Waypoint(FieldConfig.getInstance().NOTE_AMP_SIDE().plus(new Translation2d(-Units.feetToMeters(4.0), -Units.feetToMeters(2.0))))
        );
    }

    @Override
    public Command routine() {
        return Commands.sequence(
            drive(driveAndShootPreload, () -> RobotContainer.drive.getPose().getX() > FieldConfig.getInstance().NOTE_AMP_SIDE().getX() - Units.feetToMeters(1.0)),
            decideAndFetch(grabFirstTree, () -> {
                System.out.println(RobotContainer.noteDetector.firstAvailableNote().name());
                return RobotContainer.noteDetector.firstAvailableNote();
            } , firstDiversion),
            // choose(scoreFirstTree, firstDiversion::getChoice).alongWith(RobotContainer.intake.intakeContinuously().withTimeout(1.0).andThen(RobotContainer.intake.offContinous().withTimeout(0.5))),
            waitSeconds(1.0),
            Commands.sequence(
                driveAndFetch(gobbleG, 1.0),
                // drive(scoreGAndGetK, () -> RobotContainer.drive.getPose().getX() < FieldConfig.getInstance().NOTE_AMP_SIDE().getX() - Units.feetToMeters(3.0)).alongWith(RobotContainer.intake.intakeContinuously().withTimeout(1.0).andThen(RobotContainer.intake.offContinous().withTimeout(0.5))),
                fetchWithTimeout(1.5)
            ).onlyIf(() -> firstDiversion.getChoice() != SIDE.RIGHT || RobotBase.isSimulation()) // if we went for H, now go for G, otherwise stop
        );
    }

    @Override
    public Route getRoute() {
        return new Route()
            .addPaths(driveAndShootPreload)
            .addTrees(grabFirstTree, scoreFirstTree)
            .addPaths(scoreGAndGetK)
            .withKnownStart();
    }
    
}