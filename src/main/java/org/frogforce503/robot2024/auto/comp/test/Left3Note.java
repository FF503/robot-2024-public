package org.frogforce503.robot2024.auto.comp.test;

import org.frogforce503.lib.auto.AutoMode;
import org.frogforce503.lib.trajectory.PlannedPath;
import org.frogforce503.lib.trajectory.Route;
import org.frogforce503.lib.trajectory.SwervePathBuilder;
import org.frogforce503.lib.trajectory.Waypoint;
import org.frogforce503.robot2024.fields.FieldConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class Left3Note extends AutoMode {

    PlannedPath grabFirst, comeBackToScoreFirst, gobbleSecond;

    public Left3Note() {
        // grabFirst = SwervePathBuilder.generate(4.0, 3.0,
        //     Waypoint.fromHolonomicPose(new Pose2d(new Translation2d(FieldConfig.getInstance().INITIATION_LINE_X - 0.6, FieldConfig.getInstance().SPIKE_CENTER.plus(FieldConfig.getInstance().SPIKE_RIGHT).getY()/2), new Rotation2d(Math.PI))),
        //     Waypoint.fromHolonomicPose(new Pose2d(FieldConfig.getInstance().CENTERSTAGE.plus(FieldConfig.getInstance().STAGE_LEFT).times(0.5), new Rotation2d(Math.PI))),
        //     Waypoint.fromHolonomicPose(new Pose2d(FieldConfig.getInstance().STAGE_LEFT.plus(FieldConfig.getInstance().STAGE_RIGHT).times(0.5), new Rotation2d(Math.PI))),
        //     Waypoint.fromHolonomicPose(new Pose2d(FieldConfig.getInstance().SHARED_C.plus(new Translation2d(-0.5, 0)), new Rotation2d(Math.PI)))
        // );

        // comeBackToScoreFirst = SwervePathBuilder.generate(4.0, 3.0,
        //     Waypoint.fromHolonomicPose(grabFirst.getFinalHolonomicPose()),
        //     Waypoint.fromHolonomicPose(new Pose2d(FieldConfig.getInstance().STAGE_LEFT.plus(FieldConfig.getInstance().STAGE_RIGHT).times(0.5), new Rotation2d(Math.PI))),
        //     Waypoint.fromHolonomicPose(new Pose2d(FieldConfig.getInstance().CENTERSTAGE.plus(FieldConfig.getInstance().STAGE_LEFT).times(0.5), new Rotation2d(Math.PI))),
        //     Waypoint.fromHolonomicPose(new Pose2d(FieldConfig.getInstance().SUBWOOFER_TOP_RIGHT.plus(FieldConfig.getInstance().SUBWOOFER_TOP_LEFT).times(0.5).plus(new Translation2d(0.75, 0)), new Rotation2d(Math.PI)))
        // );

        // gobbleSecond = SwervePathBuilder.generate(4.0, 3.0,
        //     Waypoint.fromHolonomicPose(comeBackToScoreFirst.getFinalHolonomicPose()),
        //     Waypoint.fromHolonomicPose(new Pose2d(FieldConfig.getInstance().SPIKE_CENTER, new Rotation2d(Math.PI)))
        // );
    }

    @Override
    public Command routine() {
        return drive(grabFirst, () -> Timer.getMatchTime() < 1.5)
            .deadlineWith(
                // shoot.andThen(intake)
            )
            .andThen(
                drive(comeBackToScoreFirst),
                // shoot
                drive(gobbleSecond)
                    .alongWith(
                        // deploy intake
                    )
                // wait for intake finish
                // shoot
            );
    }

    @Override
    public Route getRoute() {
        return new Route(grabFirst, comeBackToScoreFirst, gobbleSecond);
    }
}