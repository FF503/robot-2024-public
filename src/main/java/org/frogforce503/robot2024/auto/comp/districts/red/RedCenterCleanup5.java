package org.frogforce503.robot2024.auto.comp.districts.red;

import org.frogforce503.lib.auto.AutoMode;
import org.frogforce503.lib.trajectory.PlannedPath;
import org.frogforce503.lib.trajectory.Route;
import org.frogforce503.lib.trajectory.SwervePathBuilder;
import org.frogforce503.robot2024.Robot;
import org.frogforce503.robot2024.RobotContainer;
import org.frogforce503.robot2024.auto.AutoUtil.Waypoints;
import org.frogforce503.robot2024.fields.FieldConfig;
import org.frogforce503.lib.trajectory.Waypoint;
import org.frogforce503.robot2024.auto.AutoUtil.Waypoints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RedCenterCleanup5 extends AutoMode {

    PlannedPath goOutForG1, goOutForG2, comeBackScoreG, grabA, grabB, grabC, goHome;
    private double maxSpeed = 4.0;
    private double maxAccel = 3.0;

    private double hintDistance = 0.0; //TODO tune
    private double noteOffset = 0.25;

    private double steadyShotDistance = 3.711;
    private Translation2d firstShotPos;

    public RedCenterCleanup5() {
        Pose2d startingPose = setupPose(
            Waypoints.facingGoal(new Pose2d(
                FieldConfig.getInstance().RED_INITIATION_LINE + Robot.bot.fullRobotLength / 2,
                FieldConfig.getInstance().NOTE_B.getY(),
                new Rotation2d())
            )
        );

        firstShotPos = FieldConfig.getInstance().RED_SPEAKER.plus(new Translation2d(-steadyShotDistance, 0.75));
        Pose2d firstShotPose = Waypoints.facingGoal(firstShotPos);

        goOutForG1 = SwervePathBuilder.generate(4.0, 3.0, 0.0, 0.675,
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().NOTE_C.interpolate(FieldConfig.getInstance().NOTE_B, 0.375), new Rotation2d(Math.PI), firstShotPose.getRotation()),
            Waypoint.fromHolonomicPose(firstShotPose)
        );

        goOutForG2 = SwervePathBuilder.generate(4.0, 3.0, 0.675, 0.0,
            Waypoint.fromHolonomicPose(goOutForG1.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0, 1.0)), null, new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().NOTE_G)
        );

        comeBackScoreG = SwervePathBuilder.generate(4.0, 3.5,
            Waypoint.fromHolonomicPose(goOutForG2.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0, 1.0)), null, new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().NOTE_C.interpolate(FieldConfig.getInstance().NOTE_B, 0.5), new Rotation2d(), new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().RED_SPEAKER.plus(new Translation2d(-1.75, 0)))
        );

        Translation2d AToSpeaker = FieldConfig.getInstance().NOTE_A.minus(FieldConfig.getInstance().RED_SPEAKER);
        Translation2d BToSpeaker = FieldConfig.getInstance().NOTE_B.minus(FieldConfig.getInstance().RED_SPEAKER);
        Translation2d CToSpeaker = FieldConfig.getInstance().NOTE_C.minus(FieldConfig.getInstance().RED_SPEAKER);

        grabA = SwervePathBuilder.generate(maxSpeed, maxAccel, 
            Waypoint.fromHolonomicPose(comeBackScoreG.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().NOTE_A.minus(new Translation2d(noteOffset, AToSpeaker.getAngle())), new Rotation2d(Math.PI), AToSpeaker.getAngle())
        );

        grabB = SwervePathBuilder.generate(maxSpeed, maxAccel, 
            Waypoint.fromHolonomicPose(grabA.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().NOTE_A.interpolate(FieldConfig.getInstance().NOTE_B, 0.5).plus(new Translation2d(1.35, 0))),
            new Waypoint(FieldConfig.getInstance().NOTE_B.minus(new Translation2d(noteOffset, BToSpeaker.getAngle())), BToSpeaker.getAngle(), BToSpeaker.getAngle())
        );

        grabC = SwervePathBuilder.generate(maxSpeed, maxAccel, 
            Waypoint.fromHolonomicPose(grabB.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().NOTE_B.interpolate(FieldConfig.getInstance().NOTE_C, 0.5).plus(new Translation2d(1.35, 0))),
            new Waypoint(FieldConfig.getInstance().NOTE_C.minus(new Translation2d(noteOffset, CToSpeaker.getAngle())), CToSpeaker.getAngle(), CToSpeaker.getAngle())
        );
    }

    @Override
    public Command routine() {
        // TODO Auto-generated method stub
        return Commands.sequence(
            RobotContainer.shooter.rampUp(),
            Commands.deadline( // grabs G while shooting P
                Commands.sequence(
                    drive(goOutForG1, () -> RobotContainer.drive.getPose().getTranslation().getX() <= FieldConfig.getInstance().NOTE_B.getX() + 0.5 && RobotContainer.feeder.noteInEntrySensor()),
                    drive(goOutForG2)
                ),
                Commands.waitUntil(() -> RobotContainer.drive.getPose().getTranslation().getX() <= firstShotPos.getX() + Units.feetToMeters(1.0))
                    .deadlineWith(RobotContainer.hint(steadyShotDistance + Units.feetToMeters(1.0)))
                    .andThen(
                        RobotContainer.shootSequence(),
                        RobotContainer.intake().alongWith(RobotContainer.wrist.intakeAssist())
                    )
            ),
            driveAndIntake(comeBackScoreG, () -> true)
                .deadlineWith(
                    RobotContainer.hint(1.4),
                    Commands.print("RUNNING BACK PATH").repeatedly()
                ),
            RobotContainer.shootSequence(),
            driveAndIntake(grabA, () -> true).withTimeout(3).deadlineWith(RobotContainer.hint(hintDistance)),
            Commands.waitSeconds(0.5),
            RobotContainer.shootSequence(),
            driveAndIntake(grabB, () -> true).withTimeout(3),
            Commands.waitSeconds(0.5),
            RobotContainer.shootSequence(),
            driveAndIntake(grabC, () -> true).withTimeout(3),
            Commands.waitSeconds(0.5),
            RobotContainer.shootSequence()
        );
        
    }

    @Override
    public Route getRoute() {
        // TODO Auto-generated method stub
        return new Route(goOutForG1, goOutForG2, comeBackScoreG, grabA, grabB, grabC);
    }
    
}
