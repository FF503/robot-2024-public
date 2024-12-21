package org.frogforce503.robot2024.auto.comp.WorkingAutons.blue;

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
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class BlueCenter4OFSZ extends AutoMode {

    private PlannedPath grabJ, grabE, grabF, grabG, scoreE, scoreF, scoreG;
    private HashMap<CENTERLINE_NOTES, PlannedPath> grabPath = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> scorePaths = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, Pair<Double, Double>> aimMap = new HashMap<>();


    private double maxVel = 4.5;
    private double maxAcc = 4;
    
    Translation2d shootingPos;

    private CENTERLINE_NOTES note1, note2;

    public BlueCenter4OFSZ (CENTERLINE_NOTES note1, CENTERLINE_NOTES note2) {

        Pose2d startingPose = setupPose(
            Waypoints.facingGoal(FieldConfig.getInstance().NOTE_J.minus(new Translation2d(1.5, 0)))
        );

        shootingPos = FieldConfig.getInstance().BLUE_STAGE_LEFT.interpolate(FieldConfig.getInstance().BLUE_CENTER_STAGE, 0.5).plus(new Translation2d(-0.5, 1));
        Pose2d shootingPose = Waypoints.facingGoal(shootingPos).transformBy(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(3.5)));


        aimMap.put(CENTERLINE_NOTES.E, Pair.of(0.0, 0.0));
        aimMap.put(CENTERLINE_NOTES.F, Pair.of(0.0, 0.0));
        aimMap.put(CENTERLINE_NOTES.G, Pair.of(0.0, 0.0));

        
        grabJ = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().NOTE_J, new Rotation2d(), new Rotation2d()),
            Waypoint.fromHolonomicPose(shootingPose)
        );

        grabE = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(grabJ.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_LEFT.interpolate(FieldConfig.getInstance().BLUE_CENTER_STAGE, 0.5)),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_LEFT.interpolate(FieldConfig.getInstance().BLUE_STAGE_RIGHT, 0.5)),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_RIGHT.plus(new Translation2d(0.75, 0.75)), null, Rotation2d.fromDegrees(-45)),
            new Waypoint(FieldConfig.getInstance().NOTE_E)
        );

        grabF = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(grabJ.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_LEFT.interpolate(FieldConfig.getInstance().BLUE_CENTER_STAGE, 0.5)),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_LEFT.interpolate(FieldConfig.getInstance().BLUE_STAGE_RIGHT, 0.5)),
            new Waypoint(FieldConfig.getInstance().NOTE_F, null, new Rotation2d())
        );


        grabG = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(grabJ.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_LEFT.plus(new Translation2d(0, 1))),
            new Waypoint(FieldConfig.getInstance().NOTE_G, null, Rotation2d.fromDegrees(-30))
        );
       

        grabPath.put(CENTERLINE_NOTES.E, grabE);
        grabPath.put(CENTERLINE_NOTES.F, grabF);
        grabPath.put(CENTERLINE_NOTES.G, grabG);


        scoreE = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(grabE.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_LEFT.interpolate(FieldConfig.getInstance().BLUE_STAGE_RIGHT, 0.5)),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_LEFT.interpolate(FieldConfig.getInstance().BLUE_CENTER_STAGE, 0.5)),
            Waypoint.fromHolonomicPose(shootingPose)
        );


        scoreF = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(grabF.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_LEFT.interpolate(FieldConfig.getInstance().BLUE_STAGE_RIGHT, 0.5), null, new Rotation2d()),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_LEFT.interpolate(FieldConfig.getInstance().BLUE_CENTER_STAGE, 0.5), null, null),
            Waypoint.fromHolonomicPose(shootingPose)
        );


        scoreG = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(grabG.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_LEFT.plus(new Translation2d(0, 1)), null, new Rotation2d()),
            Waypoint.fromHolonomicPose(shootingPose)
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
            // RobotContainer.setArmAndWristDown(), // TODO: accounts for bad pid
            RobotContainer.hintAng(0, 23),
            RobotContainer.shooter.rampUp(),
            RobotContainer.shootSequence(),

            driveAndIntake(grabJ).deadlineWith(RobotContainer.hintAng(0, 0), RobotContainer.startAim().beforeStarting(waitSeconds(0.075))),
            RobotContainer.shootSequence(),

            driveAndFetchFixed(grabPath.get(this.note1), 0.75)
                .deadlineWith(RobotContainer.wrist.setWristDown()).withTimeout(grabPath.get(this.note1).getTotalTimeSeconds()),
            driveAndIntake(scorePaths.get(this.note1))
                .deadlineWith(RobotContainer.hintAng(aimMap.get(this.note1))),
            
            RobotContainer.shootSequence(),

            driveAndFetchFixed(grabPath.get(this.note2), 1.0)
            .deadlineWith(RobotContainer.wrist.setWristDown()).withTimeout(grabPath.get(this.note2).getTotalTimeSeconds()),
            driveAndIntake(scorePaths.get(this.note2))
                .deadlineWith(RobotContainer.hintAng(aimMap.get(this.note2))),
            
            RobotContainer.shootSequence()
        );
    }

    @Override
    public Route getRoute() {
        return new Route()
            .addPaths(grabJ, grabPath.get(this.note1), scorePaths.get(this.note1), grabPath.get(this.note2), scorePaths.get(this.note2));
    }
}