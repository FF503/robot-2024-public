package org.frogforce503.robot2024.auto.comp.test;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.frogforce503.lib.auto.AutoMode;
import org.frogforce503.lib.trajectory.PlannedPath;
import org.frogforce503.lib.trajectory.Route;
import org.frogforce503.lib.trajectory.SwervePathBuilder;
import org.frogforce503.lib.trajectory.Waypoint;
import org.frogforce503.lib.trajectory.Waypoint.Tag;
import org.frogforce503.robot2024.auto.AutoUtil;
import org.frogforce503.robot2024.fields.FieldConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;

public class Center4Note extends AutoMode {

    Tag diversionPoint = new Tag();
    PlannedPath getLeftPieceA, getCenterPieceA, getRightPieceA;
    PlannedPath[] pickupA;
    
    PlannedPath scoreLeftPieceA, scoreCenterPieceA, scoreRightPieceA;
    PlannedPath[] scoreA;

    Tag secondDiversion = new Tag();
    PlannedPath getLeftPieceB, getCenterPieceB, getRightPieceB;
    PlannedPath[] pickupB;

    Tag thirdDiversion = new Tag();

    public Center4Note(Pose2d startingPose) {
        
        // if (RobotBase.isSimulation())
        //     startingPose = AutoUtil.Waypoints.facingGoal(new Pose2d(FieldConfig.getInstance().SPIKE_RIGHT.plus(
        //         new Translation2d(-1.5, -1.5)
        //     ), new Rotation2d()));

        // getCenterPieceA = SwervePathBuilder.generate(5.0, 4.0,
        //     Waypoint.fromHolonomicPose(startingPose),
        //     new Waypoint(FieldConfig.getInstance().CENTERSTAGE.interpolate(FieldConfig.getInstance().STAGE_RIGHT, 0.35), null, new Rotation2d()),
        //     new Waypoint(FieldConfig.getInstance().STAGE_LEFT.interpolate(FieldConfig.getInstance().STAGE_RIGHT, 0.5).plus(new Translation2d(-0.3, 0.0)), new Rotation2d(), new Rotation2d())
        //         .tagged(diversionPoint),
        //     new Waypoint(FieldConfig.getInstance().SHARED_C.minus(new Translation2d(0.5, 0.0)))
        // );

        // getLeftPieceA = SwervePathBuilder.generate(5.0, 4.0,
        //     new ArrayList<Waypoint>() {{
        //         Translation2d target = FieldConfig.getInstance().SHARED_D;
        //         List<Waypoint> beginning = getCenterPieceA.getWaypoints().subList(0, 3);
        //         Rotation2d pickupAngle = target.minus(beginning.get(beginning.size() - 1).getTranslation()).getAngle();
        //         addAll(beginning);
        //         add(new Waypoint(target.minus(new Translation2d(0.5, pickupAngle)), pickupAngle, pickupAngle));
        //     }}
        // );

        // getRightPieceA = SwervePathBuilder.generate(5.0, 4.0,
        //     new ArrayList<Waypoint>() {{
        //         Translation2d target = FieldConfig.getInstance().SHARED_B;
        //         List<Waypoint> beginning = getCenterPieceA.getWaypoints().subList(0, 3);
        //         Rotation2d pickupAngle = target.minus(beginning.get(beginning.size() - 1).getTranslation()).getAngle();
        //         addAll(beginning);
        //         add(new Waypoint(target.minus(new Translation2d(0.5, pickupAngle)), pickupAngle, pickupAngle));
        //     }}
        // );

        // pickupA = new PlannedPath[] { getLeftPieceA, getRightPieceA, getCenterPieceA };

        // Pose2d firstShootingPosition = AutoUtil.Waypoints.facingGoal(
        //     new Pose2d(FieldConfig.getInstance().STAGE_LEFT.interpolate(FieldConfig.getInstance().CENTERSTAGE, 0.5).plus(new Translation2d(-0.5, 1.25)), new Rotation2d())
        // );

        // scoreLeftPieceA = SwervePathBuilder.generate(5.0, 4.0,
        //     Waypoint.fromHolonomicPose(getLeftPieceA.getFinalHolonomicPose()),
        //     new Waypoint(FieldConfig.getInstance().STAGE_LEFT.plus(new Translation2d(0, 0.5)), null, new Rotation2d()),
        //     Waypoint.fromHolonomicPose(firstShootingPosition)
        //     // new Waypoint(FieldConfig.getInstance().STAGE_LEFT.plus(new Translation2d(-1.5, 0.8)), null, new Rotation2d())
        // );

        // scoreCenterPieceA = SwervePathBuilder.generate(5.0, 4.0,
        //     Waypoint.fromHolonomicPose(getCenterPieceA.getFinalHolonomicPose()),
        //     // new Waypoint(FieldConfig.getInstance().STAGE_LEFT.interpolate(FieldConfig.getInstance().STAGE_RIGHT, 0.5), new Rotation2d(Math.PI), new Rotation2d()),
        //     new Waypoint(FieldConfig.getInstance().STAGE_LEFT.interpolate(FieldConfig.getInstance().CENTERSTAGE, 0.4), null, new Rotation2d()),
        //     // new Waypoint(FieldConfig.getInstance().STAGE_LEFT.plus(FieldConfig.getInstance().CENTERSTAGE).times(0.5).plus(new Translation2d(-0.5, 0.5)), null, new Rotation2d())
        //     Waypoint.fromHolonomicPose(firstShootingPosition)
        // );

        // scoreRightPieceA = SwervePathBuilder.generate(5.0, 4.0,
        //     Waypoint.fromHolonomicPose(getRightPieceA.getFinalHolonomicPose()),
        //     // new Waypoint(FieldConfig.getInstance().STAGE_LEFT.interpolate(FieldConfig.getInstance().STAGE_RIGHT, 0.5), new Rotation2d(Math.PI), new Rotation2d()),
        //     new Waypoint(FieldConfig.getInstance().STAGE_LEFT.interpolate(FieldConfig.getInstance().CENTERSTAGE, 0.4), null, new Rotation2d()),
        //     Waypoint.fromHolonomicPose(firstShootingPosition)
        //     // new Waypoint(FieldConfig.getInstance().STAGE_LEFT.plus(FieldConfig.getInstance().CENTERSTAGE).times(0.5).plus(new Translation2d(-0.5, 0.5)), null, new Rotation2d())
        // );

        // scoreA = new PlannedPath[] { scoreLeftPieceA, scoreRightPieceA, scoreCenterPieceA }; // order MUST match pickup order

        // getCenterPieceB = SwervePathBuilder.generate(5.0, 4.0,
        //     Waypoint.fromHolonomicPose(scoreCenterPieceA.getFinalHolonomicPose()),
        //     // new Waypoint(FieldConfig.getInstance().CENTERSTAGE.interpolate(FieldConfig.getInstance().STAGE_LEFT, 0.6), null, new Rotation2d()),
        //     new Waypoint(FieldConfig.getInstance().STAGE_LEFT.interpolate(FieldConfig.getInstance().STAGE_RIGHT, 0.35).plus(new Translation2d(-0.3, 0.125)), new Rotation2d(), new Rotation2d())
        //         .tagged(secondDiversion),
        //     new Waypoint(FieldConfig.getInstance().SHARED_C.minus(new Translation2d(0.5, 0.0)))
        // );

        // getLeftPieceB = SwervePathBuilder.generate(5.0, 4.0,
        //     new ArrayList<Waypoint>() {{
        //         Translation2d target = FieldConfig.getInstance().SHARED_D;
        //         List<Waypoint> beginning = getCenterPieceB.getWaypoints().subList(0, 2);
        //         Rotation2d pickupAngle = target.minus(beginning.get(beginning.size() - 1).getTranslation()).getAngle();
        //         addAll(beginning);
        //         add(new Waypoint(target.minus(new Translation2d(0.5, pickupAngle)), pickupAngle, pickupAngle));
        //     }}
        // );

        // getRightPieceB = SwervePathBuilder.generate(5.0, 4.0,
        //     new ArrayList<Waypoint>() {{
        //         Translation2d target = FieldConfig.getInstance().SHARED_B;
        //         List<Waypoint> beginning = getCenterPieceB.getWaypoints().subList(0, 2);
        //         Rotation2d pickupAngle = target.minus(beginning.get(beginning.size() - 1).getTranslation()).getAngle();
        //         addAll(beginning);
        //         add(new Waypoint(target.minus(new Translation2d(0.5, pickupAngle)), pickupAngle, pickupAngle));
        //     }}
        // );

        // pickupB = new PlannedPath[] { getLeftPieceB, getRightPieceB, getCenterPieceB }; // order MUST match pickup order
    }

    ArrayList<Integer> options = new ArrayList<>(List.of(0, 1, 2));
    private int getRandomPiece() {
        int idx = (int)(Math.random() * options.size());
        int value = options.get(idx);
        options.remove(idx);
        System.out.println("chose " + value);
        return value;
    }


    @Override
    public Command routine() {
        return null;
        // return decide(pickupA, diversionPoint, 0.5, this::getRandomPiece)
            // .andThen(
                // choose(scoreA, diversionPoint::getChoice),
                // waitSeconds(0.5),
                // decide(pickupB, secondDiversion, 0.5, this::getRandomPiece),
                // choose(scoreA, secondDiversion::getChoice),
                // waitSeconds(0.5),
                // decide(pickupB, thirdDiversion, 0.5, this::getRandomPiece),
                // choose(scoreA, thirdDiversion::getChoice)
            // );
    }

    @Override
    public Route getRoute() {
        return new Route().addPaths(pickupA);
    }
    
}