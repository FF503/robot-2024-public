package org.frogforce503.robot2024.auto.comp.districts.blue;

import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import org.frogforce503.lib.auto.AutoMode;
import org.frogforce503.lib.trajectory.PlannedPath;
import org.frogforce503.lib.trajectory.Route;
import org.frogforce503.lib.trajectory.SwervePathBuilder;
import org.frogforce503.lib.trajectory.Waypoint;
import org.frogforce503.robot2024.Robot;
import org.frogforce503.robot2024.RobotContainer;
import org.frogforce503.robot2024.auto.AutoUtil.Waypoints;
import org.frogforce503.robot2024.fields.FieldConfig;
import org.frogforce503.robot2024.planners.ShotPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class BlueCenter4Note extends AutoMode{

    private PlannedPath GrabJ, GrabE, comeBackScoreE, GrabF, comeBackScoreF, goOutForMore;
    private double hint1Distance = 0;

    public BlueCenter4Note() {
        Pose2d startingPose = setupPose(
            Waypoints.facingGoal(new Pose2d(
                FieldConfig.getInstance().BLUE_INITIATION_LINE - Robot.bot.fullRobotLength / 2,
                FieldConfig.getInstance().NOTE_J.getY(),
                new Rotation2d()
            ))
        );

        Pose2d leftShootPos = Waypoints.facingGoal(
            new Pose2d(FieldConfig.getInstance().NOTE_J.plus(new Translation2d(Units.feetToMeters(0.5), 0)),
            new Rotation2d()
        ));

        GrabJ = SwervePathBuilder.generate(4.5,3.75, 
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().NOTE_J),
            Waypoint.fromHolonomicPose(leftShootPos)
        );

        hint1Distance = ShotPlanner.getInstance().getRawDistanceToGoal(leftShootPos.getTranslation());

        Rotation2d directionThru = FieldConfig.getInstance().BLUE_STAGE_LEFT.minus(FieldConfig.getInstance().BLUE_CENTER_STAGE).getAngle().plus(new Rotation2d(-Math.PI/2));

        GrabE = SwervePathBuilder.generate(4.5, 3.75,
            Waypoint.fromHolonomicPose(GrabJ.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().BLUE_CENTER_STAGE.interpolate(FieldConfig.getInstance().BLUE_STAGE_LEFT, 0.5), null, new Rotation2d()),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_LEFT.interpolate(FieldConfig.getInstance().BLUE_STAGE_RIGHT, 0.5), null, new Rotation2d()),           
            new Waypoint(FieldConfig.getInstance().NOTE_E, new Rotation2d(), new Rotation2d())
        );

        comeBackScoreE = SwervePathBuilder.generate(5, 4,
            Waypoint.fromHolonomicPose(GrabE.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_LEFT.interpolate(FieldConfig.getInstance().BLUE_STAGE_RIGHT, 0.5), null, new Rotation2d()),
            new Waypoint(FieldConfig.getInstance().BLUE_CENTER_STAGE.interpolate(FieldConfig.getInstance().BLUE_STAGE_LEFT, 0.6), null, new Rotation2d()),
            Waypoint.fromHolonomicPose(leftShootPos)
        );
        
        GrabF = SwervePathBuilder.generate(4.5, 3.75,
            Waypoint.fromHolonomicPose(comeBackScoreE.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().BLUE_CENTER_STAGE.interpolate(FieldConfig.getInstance().BLUE_STAGE_LEFT, 0.5), null, new Rotation2d()),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_LEFT.interpolate(FieldConfig.getInstance().BLUE_STAGE_RIGHT, 0.5), null, new Rotation2d()),
            new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(-Units.feetToMeters(2), 0)))
        );

        comeBackScoreF = SwervePathBuilder.generate(5, 4,
            Waypoint.fromHolonomicPose(GrabF.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_LEFT.interpolate(FieldConfig.getInstance().BLUE_STAGE_RIGHT, 0.5), null, new Rotation2d()),
            new Waypoint(FieldConfig.getInstance().BLUE_CENTER_STAGE.interpolate(FieldConfig.getInstance().BLUE_STAGE_LEFT, 0.6), null, new Rotation2d()),
            Waypoint.fromHolonomicPose(leftShootPos)
        );

        goOutForMore = SwervePathBuilder.generate(4.5, 3.75, 0, 3.5,
            Waypoint.fromHolonomicPose(comeBackScoreF.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().BLUE_CENTER_STAGE.interpolate(FieldConfig.getInstance().BLUE_STAGE_LEFT, 0.5), null, new Rotation2d()),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_LEFT.interpolate(FieldConfig.getInstance().BLUE_STAGE_RIGHT, 0.5), null, new Rotation2d())
        );
    }

    @Override
    public Command routine() {
        return Commands.sequence(
            RobotContainer.shootSequence(),
            driveAndIntake(GrabJ)
                .deadlineWith(RobotContainer.hintAng(0, 5))
                .withTimeout(GrabJ.getTotalTimeSeconds() - 0.5), // because this path ends wonky
            Commands.print("SHOOT SEQUENCE"),
            RobotContainer.shootSequence(),
            driveAndFetchFixed(GrabE, 0.85),
            drive(comeBackScoreE)
                .deadlineWith(
                    RobotContainer.hintAng(0, 6),
                    RobotContainer.startAim(),
                    RobotContainer.intake()
                ),
            RobotContainer.shootSequence(),
            driveAndFetchFixed(GrabF, 0.85),
            drive(comeBackScoreF)
                .deadlineWith(
                    RobotContainer.intake(),
                    RobotContainer.hintAng(0, 6)
                ),
            RobotContainer.shootSequence(),
            drive(goOutForMore)
                // Commands.deferredProxy(() -> driveAndIntake(GrabJ)).until(RobotContainer.shooter::noteInCorrectPosition)
                //     .andThen(
                //         print("INTAKING SEQUENCE ENDED"),
                //         // Commands.deferredProxy(() -> {
                //         RobotContainer.aim().andThen(RobotContainer.shoot())
                //         // })
                //     )
                // drive(GrabJ)
                //     .alongWith(RobotContainer.intake())
            // waitSeconds(1.0)
            // driveAndFetch(GrabF, 0.5),
            // drive(comeBackScoreF),
            // waitSeconds(1.0)
        );
    }

    @Override
    public Route getRoute() {
        return new Route(GrabJ, GrabE, comeBackScoreE, GrabF, comeBackScoreF, goOutForMore);
    }
    
}