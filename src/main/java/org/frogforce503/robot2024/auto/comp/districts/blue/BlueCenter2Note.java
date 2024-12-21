package org.frogforce503.robot2024.auto.comp.districts.blue;

import org.frogforce503.lib.auto.AutoMode;
import org.frogforce503.lib.trajectory.PlannedPath;
import org.frogforce503.lib.trajectory.Route;
import org.frogforce503.lib.trajectory.SwervePathBuilder;
import org.frogforce503.lib.trajectory.Waypoint;
import org.frogforce503.robot2024.Robot;
import org.frogforce503.robot2024.auto.AutoUtil.Waypoints;
import org.frogforce503.robot2024.fields.FieldConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class BlueCenter2Note extends AutoMode{

    private PlannedPath GrabJ;

    public BlueCenter2Note() {
        Pose2d startingPose = setupPose(Waypoints.facingGoal(
            new Pose2d(
                FieldConfig.getInstance().INITIATION_LINE_X() - Robot.bot.fullRobotLength / 2,
                FieldConfig.getInstance().NOTE_CENTER().getY(),
                new Rotation2d()
            ))
        );

        GrabJ = SwervePathBuilder.generate(5, 3, 
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().NOTE_CENTER())
        );
    }

    @Override
    public Command routine() {
        // TODO Auto-generated method stub
        return drive(GrabJ);
    }

    @Override
    public Route getRoute() {
        return new Route(GrabJ);
    }
    
}