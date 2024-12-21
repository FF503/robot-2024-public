package org.frogforce503.robot2024.auto.comp.WorkingAutons.red;

import java.util.HashMap;
import org.frogforce503.lib.auto.AutoMode;
import org.frogforce503.lib.auto.AutoChooser.CENTERLINE_NOTES;
import org.frogforce503.lib.trajectory.PlannedPath;
import org.frogforce503.lib.trajectory.Route;
import org.frogforce503.lib.trajectory.SwervePathBuilder;
import org.frogforce503.lib.trajectory.Waypoint;
import org.frogforce503.robot2024.RobotContainer;
import org.frogforce503.robot2024.auto.AutoUtil.Waypoints;
import org.frogforce503.robot2024.fields.FieldConfig;
import org.frogforce503.robot2024.planners.ShotPlanner.Presets;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RedCenter4OFSZ extends AutoMode {

    private PlannedPath grabB;
    private HashMap<CENTERLINE_NOTES, PlannedPath> grabPath = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> scorePaths = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, Pair<Double, Double>> aimMap = new HashMap<>();

    private double maxVel = 4.8;
    private double maxAcc = 4.75;

    private CENTERLINE_NOTES note1, note2;

    public RedCenter4OFSZ (CENTERLINE_NOTES note1, CENTERLINE_NOTES note2) {

        Pose2d startingPose = setupPose(
            Waypoints.facingGoal(FieldConfig.getInstance().NOTE_B.plus(new Translation2d(1.5, 0)))
        );

        aimMap.put(CENTERLINE_NOTES.E, Pair.of(0.0, 0.0));
        aimMap.put(CENTERLINE_NOTES.F, Pair.of(0.0, 0.0));
        aimMap.put(CENTERLINE_NOTES.G, Pair.of(0.0, 0.0));

        grabB = SwervePathBuilder.generate(4.0, 3.0,
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().NOTE_B, null, new Rotation2d(Math.PI))
        );

        var grabE = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(grabB.getFinalHolonomicPose()).withHolonomicRotation(new Rotation2d(Math.PI).plus(Rotation2d.fromDegrees(15))),
            new Waypoint(FieldConfig.getInstance().NOTE_E, null, new Rotation2d(Math.PI).plus(Rotation2d.fromDegrees(15)))
        );

        var grabF = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(grabB.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(3.0, 0.0)), null, new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().NOTE_F, null, new Rotation2d(Math.PI))
        );

        var grabG = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(grabB.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0.0, 1.0))),
            new Waypoint(FieldConfig.getInstance().NOTE_G, null, new Rotation2d(Math.PI))
        );

        grabPath.put(CENTERLINE_NOTES.E, grabE);
        grabPath.put(CENTERLINE_NOTES.F, grabF);
        grabPath.put(CENTERLINE_NOTES.G, grabG);

        var scoreE = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(grabE.getFinalHolonomicPose()).withHolonomicRotation(new Rotation2d(Math.PI)),
            Waypoint.fromHolonomicPose(grabB.getFinalHolonomicPose())
        );

        var scoreF = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(grabF.getFinalHolonomicPose()).withHolonomicRotation(new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(3.0, 0.0)), null, new Rotation2d(Math.PI)),
            Waypoint.fromHolonomicPose(grabB.getFinalHolonomicPose())
        );

        var scoreG = SwervePathBuilder.generate(4.0, 4.75,
            Waypoint.fromHolonomicPose(grabG.getFinalHolonomicPose()).withHolonomicRotation(new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0.0, 1.0))),
            Waypoint.fromHolonomicPose(grabB.getFinalHolonomicPose())
        );

        scorePaths.put(CENTERLINE_NOTES.E, scoreE);
        scorePaths.put(CENTERLINE_NOTES.F, scoreF);
        scorePaths.put(CENTERLINE_NOTES.G, scoreG);

        this.note1 = note1;
        this.note2 = note2;
    }

    @Override
    public CENTERLINE_NOTES[] priorityList() {
        return new CENTERLINE_NOTES[] {note1, note2};
    }

    @Override
    public Command routine() {
        return Commands.sequence(
            RobotContainer.selectPreset(Presets.BATTER),
            RobotContainer.shootSequence(),

            driveAndIntake(grabB).deadlineWith(RobotContainer.hintAuton(0.0, 5.5)),
            RobotContainer.shootSequence().beforeStarting(RobotContainer.hintAuton(0.0, 5.5, 0.05)),

            driveAndFetchFixed(grabPath.get(this.note1), 0.25),
            driveAndIntake(scorePaths.get(this.note1)).deadlineWith(RobotContainer.hintAuton(0.0, 5.5)),
            
            RobotContainer.shootSequence().beforeStarting(RobotContainer.hintAuton(0.0, 5.5, 0.05)),

            driveAndFetchFixed(grabPath.get(this.note2), 0.25),
            driveAndIntake(scorePaths.get(this.note2)).deadlineWith(RobotContainer.hintAuton(0.0, 5.5)),
            
            RobotContainer.shootSequence().beforeStarting(RobotContainer.hintAuton(0.0, 5.5, 0.05))
        );
    }

    @Override
    public Route getRoute() {
        return new Route()
            .addPaths(grabB, grabPath.get(this.note1), scorePaths.get(this.note1), grabPath.get(this.note2), scorePaths.get(this.note2));
    }
}