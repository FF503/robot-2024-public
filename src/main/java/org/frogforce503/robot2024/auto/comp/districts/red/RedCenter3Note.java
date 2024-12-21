package org.frogforce503.robot2024.auto.comp.districts.red;

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

public class RedCenter3Note extends AutoMode{

    private PlannedPath GrabB, GrabF, comeBackScoreF;
    private double hint1Distance = 0;

    public RedCenter3Note() {
        Pose2d startingPose = setupPose(
            // Waypoints.facingGoal(new Pose2d(
                new Pose2d(FieldConfig.getInstance().RED_INITIATION_LINE + Robot.bot.fullRobotLength / 2,
                FieldConfig.getInstance().NOTE_B.getY(),
                new Rotation2d(Math.PI))
            // ))
        );

        Pose2d leftShootPos =// Waypoints.facingGoal(
            new Pose2d(FieldConfig.getInstance().NOTE_B.minus(new Translation2d(Units.feetToMeters(0.5), 0)),
            new Rotation2d(Math.PI));
        // ));

        System.out.println(startingPose);

        GrabB = SwervePathBuilder.generate(4.0,3.5, 
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().NOTE_B),
            Waypoint.fromHolonomicPose(leftShootPos)
        );

        hint1Distance = ShotPlanner.getInstance().getRawDistanceToGoal(leftShootPos.getTranslation()) + Units.feetToMeters(0.5);

        GrabF = SwervePathBuilder.generate(4.5, 3.75,
            Waypoint.fromHolonomicPose(GrabB.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().RED_CENTER_STAGE.interpolate(FieldConfig.getInstance().RED_STAGE_RIGHT, 0.375), null, new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.interpolate(FieldConfig.getInstance().RED_STAGE_LEFT, 0.5), new Rotation2d(Math.PI), new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(Units.feetToMeters(2), 0)))
        );

        comeBackScoreF = SwervePathBuilder.generate(5, 4,
            Waypoint.fromHolonomicPose(GrabF.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.interpolate(FieldConfig.getInstance().RED_STAGE_LEFT, 0.5), null, new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().RED_CENTER_STAGE.interpolate(FieldConfig.getInstance().RED_STAGE_RIGHT, 0.6), null, new Rotation2d(Math.PI)),
            Waypoint.fromHolonomicPose(leftShootPos)
        );
    }

    @Override
    public Command routine() {
        return Commands.sequence(
            RobotContainer.shootSequence(),
            driveAndIntake(GrabB)
                .deadlineWith(RobotContainer.hint(hint1Distance)),
            RobotContainer.shootSequence(),
            driveAndIntake(GrabF),
            drive(comeBackScoreF)
                .deadlineWith(
                    RobotContainer.intake(),
                    RobotContainer.hint(hint1Distance)
                ),
            RobotContainer.shootSequence()
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
        return new Route(GrabB, GrabF, comeBackScoreF);
    }
    
}