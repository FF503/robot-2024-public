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

public class BlueCenter3Note extends AutoMode{

    private PlannedPath GrabJ, GrabF, comeBackScoreF;
    private double hint1Distance = 0;

    public BlueCenter3Note() {
        Pose2d startingPose = setupPose(
            Waypoints.facingGoal(new Pose2d(
                FieldConfig.getInstance().INITIATION_LINE_X() - Robot.bot.fullRobotLength / 2,
                FieldConfig.getInstance().NOTE_CENTER().getY(),
                new Rotation2d()
            ))
        );

        Pose2d leftShootPos = Waypoints.facingGoal(
            new Pose2d(FieldConfig.getInstance().NOTE_CENTER().plus(new Translation2d(Units.feetToMeters(0.5), 0)),
            new Rotation2d()
        ));

        GrabJ = SwervePathBuilder.generate(4.5,3.75, 
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().NOTE_CENTER()),
            Waypoint.fromHolonomicPose(leftShootPos)
        );

        hint1Distance = ShotPlanner.getInstance().getRawDistanceToGoal(leftShootPos.getTranslation()) + Units.feetToMeters(0.5);

        Rotation2d directionThru = FieldConfig.getInstance().STAGE_AMP_SIDE().minus(FieldConfig.getInstance().CENTERSTATE()).getAngle().plus(new Rotation2d(-Math.PI/2));

        GrabF = SwervePathBuilder.generate(4.5, 3.75,
            Waypoint.fromHolonomicPose(GrabJ.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().CENTERSTATE().interpolate(FieldConfig.getInstance().STAGE_AMP_SIDE(), 0.5), null, new Rotation2d()),
            new Waypoint(FieldConfig.getInstance().STAGE_AMP_SIDE().interpolate(FieldConfig.getInstance().STAGE_SOURCE_SIDE(), 0.5), null, new Rotation2d()),
            new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(-Units.feetToMeters(2), 0)))
        );

        comeBackScoreF = SwervePathBuilder.generate(5, 4,
            Waypoint.fromHolonomicPose(GrabF.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().STAGE_AMP_SIDE().interpolate(FieldConfig.getInstance().STAGE_SOURCE_SIDE(), 0.5), null, new Rotation2d()),
            new Waypoint(FieldConfig.getInstance().CENTERSTATE().interpolate(FieldConfig.getInstance().STAGE_AMP_SIDE(), 0.6), null, new Rotation2d()),
            Waypoint.fromHolonomicPose(leftShootPos)
        );
    }

    @Override
    public Command routine() {
        return Commands.sequence(
            RobotContainer.shootSequence(),
            driveAndIntake(GrabJ)
                .deadlineWith(RobotContainer.autonHint(hint1Distance)),
            RobotContainer.shootSequenceIgnoreAngle(),
            driveAndFetch(GrabF, 0.75),
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
        return new Route(GrabJ, GrabF, comeBackScoreF);
    }
    
}