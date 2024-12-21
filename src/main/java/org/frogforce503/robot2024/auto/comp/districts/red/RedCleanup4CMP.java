package org.frogforce503.robot2024.auto.comp.districts.red;

import static edu.wpi.first.wpilibj2.command.Commands.deadline;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import java.util.HashMap;

import org.frogforce503.lib.auto.AutoChooser.CENTERLINE_NOTES;
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

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;

public class RedCleanup4CMP extends AutoMode {

    PlannedPath grabA, grabB, grabC, grabF;
    private double maxSpeed = 4.0;
    private double maxAccel = 3.0;

    private double hintDistance = 0.0; //TODO tune
    private double noteOffset = 0.35;

    public RedCleanup4CMP() {
        Pose2d startingPose = setupPose(
            Waypoints.facingGoal(new Pose2d(
                FieldConfig.getInstance().RED_INITIATION_LINE + Robot.bot.fullRobotLength / 2,
                FieldConfig.getInstance().RED_SPEAKER.getY(),
                new Rotation2d())
            )
        );

        Translation2d AToSpeaker = FieldConfig.getInstance().NOTE_A.minus(FieldConfig.getInstance().RED_SPEAKER);
        Translation2d BToSpeaker = FieldConfig.getInstance().NOTE_B.minus(FieldConfig.getInstance().RED_SPEAKER);
        Translation2d CToSpeaker = FieldConfig.getInstance().NOTE_C.minus(FieldConfig.getInstance().RED_SPEAKER);



        grabA = SwervePathBuilder.generate(maxSpeed, maxAccel, 
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().NOTE_A.minus(new Translation2d(noteOffset + 0.15, AToSpeaker.getAngle()))/*.plus(new Translation2d(0, -Units.inchesToMeters(6)))*/, AToSpeaker.getAngle(), AToSpeaker.getAngle())
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

        grabF = SwervePathBuilder.generate(maxSpeed, maxAccel, 
            Waypoint.fromHolonomicPose(grabC.getFinalHolonomicPose()),
            // new Waypoint(FieldConfig.getInstance().RED_CENTER_STAGE.interpolate(FieldConfig.getInstance().RED_STAGE_RIGHT, 0.5)),
            // new Waypoint(FieldConfig.getInstance().RED_STAGE_LEFT.interpolate(FieldConfig.getInstance().RED_STAGE_RIGHT, 0.5)),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_LEFT.interpolate(FieldConfig.getInstance().RED_STAGE_RIGHT, 0.5).interpolate(FieldConfig.getInstance().RED_CENTER_STAGE, 0.4)),
            new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(Units.feetToMeters(1.0), 0.0)), new Rotation2d(), new Rotation2d(Math.PI))
            // new Waypoint(FieldConfig.getInstance().NOTE_C.plus(new Translation2d(2, 0)))
        );

        hintDistance = ShotPlanner.getInstance().getRawDistanceToGoal(grabA.getFinalHolonomicPose().getTranslation()) - 0.4;
    }

    @Override
    public Command routine() {
        // TODO Auto-generated method stub
        return Commands.sequence(
            RobotContainer.shootSequence(),
            waitSeconds(0.15),
            // driveAndIntake(grabA).withTimeout(4).deadlineWith(RobotContainer.hintAng(0, 5.5)),
            drive(grabA).alongWith(RobotContainer.intake()).deadlineWith(RobotContainer.hintAng(0, 5.5)).withTimeout(3),
            waitSeconds(0.15),
            RobotContainer.shootSequence().withTimeout(2.0),

            // driveAndIntake(grabB).withTimeout(4).deadlineWith(RobotContainer.hintAng(0, 4)),
            drive(grabB).alongWith(RobotContainer.intake()).deadlineWith(RobotContainer.hintAng(0, 6)).withTimeout(3),
            waitSeconds(0.15),
            RobotContainer.shootSequence().withTimeout(2.0),

            // driveAndIntake(grabC).withTimeout(4).deadlineWith(RobotContainer.hintAng(0, 5)),
            drive(grabC).alongWith(RobotContainer.intake()).deadlineWith(RobotContainer.hintAng(0, 4.2)).withTimeout(3),
            waitSeconds(0.15),
            RobotContainer.shootSequence().withTimeout(2.0),

            driveAndIntake(grabF)
        );
        
    }

    @Override
    public Route getRoute() {
        // TODO Auto-generated method stub
        return new Route(grabA, grabB, grabC, grabF);
    }
    
}
