package org.frogforce503.robot2024.auto.comp.districts.red;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import java.lang.reflect.Field;
import java.util.ArrayList;
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
import org.frogforce503.robot2024.subsystems.NoteDetector.SIDE;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RedSource2oteCMP extends AutoMode {

    // private HashMap<CENTERLINE_NOTES, PlannedPath> grabFirstPath = new HashMap<>();
    // private HashMap<CENTERLINE_NOTES, PlannedPath> grabSecondPath = new HashMap<>();
    // private HashMap<CENTERLINE_NOTES, PlannedPath> bailFromPath = new HashMap<>();
    // private HashMap<CENTERLINE_NOTES, PlannedPath> scorePath = new HashMap<>();
    // private HashMap<CENTERLINE_NOTES, Pair<Double, Double>> aimMap = new HashMap<>();

    private PlannedPath grabD, scoreD;

    //11.41, 5.9
    Waypoint.Tag tag;

    public RedSource2oteCMP() {

        var startingPose = setupPose(Waypoints.facingGoal(new Translation2d(FieldConfig.getInstance().RED_INITIATION_LINE + 0.5, FieldConfig.getInstance().NOTE_A.getY())));

        Rotation2d noteDToStart = FieldConfig.getInstance().NOTE_D.minus(startingPose.getTranslation()).getAngle();
        grabD = SwervePathBuilder.generate(4.8, 4.75, // 5.0, 4.75
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_LEFT.minus(new Translation2d(0, 1.25)), null, noteDToStart),
            // new Waypoint(new Translation2d(5, noteDToStart), noteDToStart, noteDToStart),
            // new Waypoint(new Translation2d(10, noteDToStart), noteDToStart, noteDToStart),
            new Waypoint(FieldConfig.getInstance().NOTE_D, noteDToStart, noteDToStart)
        );

        tag = new Waypoint.Tag();

        scoreD = SwervePathBuilder.reversedOf(grabD, 4.0, 3.5);

    }

    @Override
    public CENTERLINE_NOTES[] priorityList() {
        return new CENTERLINE_NOTES[] {CENTERLINE_NOTES.D};
    }

    int[] storage = {0, 0};
    int[] storage2 = {0, 0};

    @Override
    public Command routine() {
        return Commands.sequence(
            RobotContainer.hintAng(0.0, 12.0),
            RobotContainer.shootSequence(),

            driveAndFetchFixed(grabD, 0.75).withTimeout(grabD.getTotalTimeSeconds() - 0.5),
            driveAndIntake(scoreD),
            RobotContainer.shootSequence().beforeStarting(waitSeconds(0.125))
            
        );
    }

    private SIDE perspective(CENTERLINE_NOTES note) {
        return (note == CENTERLINE_NOTES.D) ? SIDE.CENTER : (note == CENTERLINE_NOTES.E ? SIDE.RIGHT : SIDE.RIGHT);
    }

    @Override
    public Route getRoute() {
        return new Route(grabD, scoreD);
            // .addPathOptions(grab)
            // // .addPathOptions(scorePath)
            // // .addPathOptions(bailFromPath)
            // .addPathOptions(grabSecondPath);
    }
    
}
