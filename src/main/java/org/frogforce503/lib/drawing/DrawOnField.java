package org.frogforce503.lib.drawing;

import org.frogforce503.robot2024.RobotContainer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;

public class DrawOnField {

    private static Trajectory getPoints(Translation2d at, int number) {
        switch (number) {
            case 1:
                return NumberTrajectories.NUMBER_1.relativeTo(new Pose2d(at, new Rotation2d()).times(-1));
            case 2:
                return NumberTrajectories.NUMBER_2.relativeTo(new Pose2d(at, new Rotation2d()).times(-1));
            case 3:
                return NumberTrajectories.NUMBER_3.relativeTo(new Pose2d(at, new Rotation2d()).times(-1));
            default:
                return null;
        }
    }

    public static void number(Translation2d at, int number) {
        Trajectory points = getPoints(at, number);
        if (points != null)
            RobotContainer.drive.getField().getObject("NUMBER_"+number).setTrajectory(points);
    }
}
