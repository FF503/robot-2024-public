package org.frogforce503.robot2024.auto.comp.districts.blue;

import static org.frogforce503.robot2024.auto.AutoUtil.Waypoints.rotation;
import static org.frogforce503.robot2024.auto.AutoUtil.Waypoints.translation2d;

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
import edu.wpi.first.wpilibj2.command.WaitCommand;


// Using 3 near notes
public class BlueCleanup extends AutoMode {

    PlannedPath grabI, grabJ, grabK, goHome;
    private double maxSpeed = 4;
    private double maxAccel = 3;

    private double hintDistance = 0.0;

    public BlueCleanup() {
        Pose2d startingPose = setupPose(
            Waypoints.facingGoal(new Pose2d(
                FieldConfig.getInstance().INITIATION_LINE_X() - Robot.bot.fullRobotLength / 2,
                FieldConfig.getInstance().NOTE_SOURCE_SIDE().getY(),
                new Rotation2d())
            )
        );

        Translation2d IToSpeaker = FieldConfig.getInstance().NOTE_SOURCE_SIDE().minus(FieldConfig.getInstance().SPEAKER_AUTON());
        Translation2d JToSpeaker = FieldConfig.getInstance().NOTE_CENTER().minus(FieldConfig.getInstance().SPEAKER_AUTON());
        Translation2d KToSpeaker = FieldConfig.getInstance().NOTE_AMP_SIDE().minus(FieldConfig.getInstance().SPEAKER_AUTON());

        grabI = SwervePathBuilder.generate(maxSpeed, maxAccel, 
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().NOTE_SOURCE_SIDE().minus(new Translation2d(Units.feetToMeters(1.25), IToSpeaker.getAngle())), IToSpeaker.getAngle(), IToSpeaker.getAngle())
        );

        hintDistance = ShotPlanner.getInstance().getRawDistanceToGoal(grabI.getFinalHolonomicPose().getTranslation());

        grabJ = SwervePathBuilder.generate(maxSpeed, maxAccel, 
            Waypoint.fromHolonomicPose(grabI.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().NOTE_SOURCE_SIDE().interpolate(FieldConfig.getInstance().NOTE_CENTER(), 0.5).minus(new Translation2d(1.25, 0))),
            new Waypoint(FieldConfig.getInstance().NOTE_CENTER().minus(new Translation2d(Units.feetToMeters(1.25), JToSpeaker.getAngle())).plus(new Translation2d(0.2, 0)), JToSpeaker.getAngle(), JToSpeaker.getAngle())
        );

        grabK = SwervePathBuilder.generate(maxSpeed, maxAccel, 
            Waypoint.fromHolonomicPose(grabJ.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().NOTE_AMP_SIDE().interpolate(FieldConfig.getInstance().NOTE_CENTER(), 0.5).minus(new Translation2d(1.25, 0))),
            new Waypoint(FieldConfig.getInstance().NOTE_AMP_SIDE().minus(new Translation2d(Units.feetToMeters(1.25), KToSpeaker.getAngle())), KToSpeaker.getAngle(), KToSpeaker.getAngle())
        );

        goHome = SwervePathBuilder.generate(maxSpeed, maxAccel, 
            Waypoint.fromHolonomicPose(grabK.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().NOTE_AMP_SIDE().minus(new Translation2d(1.75, 0)), null, new Rotation2d())
        );
    }

    @Override
    public Command routine() {
        // TODO Auto-generated method stub
        return Commands.sequence(
            RobotContainer.shootSequence(),
            driveAndIntakeParallel(grabI, () -> true).withTimeout(3).deadlineWith(RobotContainer.hint(hintDistance)), 
            Commands.waitSeconds(0.5),
            RobotContainer.shootSequence(),
            driveAndIntakeParallel(grabJ, () -> true).withTimeout(3),
            Commands.waitSeconds(0.5),
            RobotContainer.shootSequence(),
            driveAndIntakeParallel(grabK, () -> true).withTimeout(3),
            Commands.waitSeconds(0.5),
            RobotContainer.shootSequence(),
            drive(goHome)
        );
    }

    @Override
    public Route getRoute() {
        // TODO Auto-generated method stub
    return new Route(grabI, grabJ, grabK, goHome);
    }
    
}
