package org.frogforce503.robot2024.auto.comp.districts.red;

import org.frogforce503.lib.auto.AutoMode;
import org.frogforce503.lib.trajectory.PlannedPath;
import org.frogforce503.lib.trajectory.Route;
import org.frogforce503.lib.trajectory.SwervePathBuilder;
import org.frogforce503.robot2024.Robot;
import org.frogforce503.robot2024.RobotContainer;
import org.frogforce503.robot2024.auto.AutoUtil.Waypoints;
import org.frogforce503.robot2024.fields.FieldConfig;
import org.frogforce503.robot2024.planners.ShotPlanner;
import org.frogforce503.lib.trajectory.Waypoint;
import org.frogforce503.robot2024.auto.AutoUtil.Waypoints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RedCleanup extends AutoMode {

    PlannedPath grabA, grabB, grabC, goHome;
    private double maxSpeed = 4.0;
    private double maxAccel = 3.0;

    private double hintDistance = 0.0; //TODO tune
    private double noteOffset = 0.25;

    public RedCleanup() {
        Pose2d startingPose = setupPose(
            Waypoints.facingGoal(new Pose2d(
                FieldConfig.getInstance().RED_INITIATION_LINE + Robot.bot.fullRobotLength / 2,
                FieldConfig.getInstance().NOTE_A.getY(),
                new Rotation2d())
            )
        );

        Translation2d AToSpeaker = FieldConfig.getInstance().NOTE_A.minus(FieldConfig.getInstance().RED_SPEAKER);
        Translation2d BToSpeaker = FieldConfig.getInstance().NOTE_B.minus(FieldConfig.getInstance().RED_SPEAKER);
        Translation2d CToSpeaker = FieldConfig.getInstance().NOTE_C.minus(FieldConfig.getInstance().RED_SPEAKER);

        grabA = SwervePathBuilder.generate(maxSpeed, maxAccel, 
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().NOTE_A.minus(new Translation2d(noteOffset, AToSpeaker.getAngle())).plus(new Translation2d(0, -Units.inchesToMeters(6))), AToSpeaker.getAngle(), AToSpeaker.getAngle())
        );

        grabB = SwervePathBuilder.generate(maxSpeed, maxAccel, 
            Waypoint.fromHolonomicPose(grabA.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().NOTE_A.interpolate(FieldConfig.getInstance().NOTE_B, 0.5).plus(new Translation2d(1.25, 0))),
            new Waypoint(FieldConfig.getInstance().NOTE_B.minus(new Translation2d(noteOffset, BToSpeaker.getAngle())), BToSpeaker.getAngle(), BToSpeaker.getAngle())
        );

        grabC = SwervePathBuilder.generate(maxSpeed, maxAccel, 
            Waypoint.fromHolonomicPose(grabB.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().NOTE_B.interpolate(FieldConfig.getInstance().NOTE_C, 0.5).plus(new Translation2d(1.25, 0))),
            new Waypoint(FieldConfig.getInstance().NOTE_C.minus(new Translation2d(noteOffset, CToSpeaker.getAngle())), CToSpeaker.getAngle(), CToSpeaker.getAngle())
        );

        goHome = SwervePathBuilder.generate(maxSpeed, maxAccel, 
            Waypoint.fromHolonomicPose(grabC.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().NOTE_C.plus(new Translation2d(2, 0)))
        );

        hintDistance = ShotPlanner.getInstance().getRawDistanceToGoal(grabA.getFinalHolonomicPose().getTranslation()) - 0.4;
    }

    @Override
    public Command routine() {
        // TODO Auto-generated method stub
        return Commands.sequence(
            RobotContainer.shootSequence(),
            driveAndIntakeParallel(grabA, () -> true).deadlineWith(RobotContainer.hint(hintDistance))
                .withTimeout(2.5),
            Commands.waitSeconds(0.5),
            RobotContainer.shootSequence(),
            driveAndIntakeParallel(grabB, () -> true).withTimeout(3.5)
                .withTimeout(2.5),
            Commands.waitSeconds(0.5),
            RobotContainer.shootSequence(),
            driveAndIntakeParallel(grabC, () -> true).withTimeout(3.5).deadlineWith(RobotContainer.hint(hintDistance + 0.4)),
            Commands.waitSeconds(0.5),
            RobotContainer.shootSequence(),
            drive(goHome)
        );
        
    }

    @Override
    public Route getRoute() {
        // TODO Auto-generated method stub
        return new Route(grabA, grabB, grabC, goHome);
    }
    
}
