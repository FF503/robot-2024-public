package org.frogforce503.robot2024.auto.comp.districts.red;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import java.util.HashMap;

// import static edu.wpi.first.units.Units.Rotation;

import org.frogforce503.lib.auto.AutoMode;
import org.frogforce503.lib.auto.AutoChooser.CENTERLINE_NOTES;
import org.frogforce503.lib.trajectory.PlannedPath;
import org.frogforce503.lib.trajectory.Route;
import org.frogforce503.lib.trajectory.SwervePathBuilder;
import org.frogforce503.lib.trajectory.Waypoint;
import org.frogforce503.robot2024.Robot;
import org.frogforce503.robot2024.RobotContainer;
import org.frogforce503.robot2024.auto.AutoUtil.Waypoints;
import org.frogforce503.robot2024.fields.FieldConfig;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RedAmp4CenterLineCMP extends AutoMode {

    private PlannedPath goOutForFirstButShoot;
    private HashMap<CENTERLINE_NOTES, PlannedPath> grabFirstPaths = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> grabSecondPaths = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> scorePaths = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, Pair<Double, Double>> aimMap = new HashMap<>();

    private final double steadyShotDistance = 3.711;

    private double maxVel = 4.8;
    private double maxAcc = 4.75;
    
    Translation2d firstShotPos;
    Translation2d secondShotPos;

    private CENTERLINE_NOTES note1, note2, note3;

    public RedAmp4CenterLineCMP(CENTERLINE_NOTES note1, CENTERLINE_NOTES note2, CENTERLINE_NOTES note3) {

        Pose2d startingPose = setupPose(
            Waypoints.facingGoal(FieldConfig.getInstance().NOTE_C.interpolate(FieldConfig.getInstance().NOTE_B, 0.5).plus(new Translation2d(1.0, 0.0)))
        );

        firstShotPos = FieldConfig.getInstance().RED_SPEAKER.minus(new Translation2d(steadyShotDistance, -0.75));
        Pose2d firstShotPose = Waypoints.facingGoal(firstShotPos);

        secondShotPos = firstShotPos.minus(new Translation2d(0.5, 0));
        Pose2d secondShotPose = Waypoints.facingGoal(secondShotPos);


        aimMap.put(CENTERLINE_NOTES.F, Pair.of(0.0, 0.0));
        aimMap.put(CENTERLINE_NOTES.G, Pair.of(0.5, 0.0));
        aimMap.put(CENTERLINE_NOTES.H, Pair.of(0.0, 0.0));

        
        goOutForFirstButShoot = SwervePathBuilder.generate(maxVel, maxAcc, 0.0, 0.675,
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().NOTE_C.interpolate(FieldConfig.getInstance().NOTE_B, 0.6), new Rotation2d(Math.PI), firstShotPose.getRotation()),
            Waypoint.fromHolonomicPose(firstShotPose.transformBy(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(-5))))
        );

        var grabGFirst = SwervePathBuilder.generate(maxVel, maxAcc, 0.675, 0.0,
            Waypoint.fromHolonomicPose(goOutForFirstButShoot.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0, 1.0)), null, new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().NOTE_G.plus(new Translation2d(Units.feetToMeters(1), -Units.feetToMeters(1.25))))
        );

        var grabFFirst = SwervePathBuilder.generate(maxVel, maxAcc, 0.675, 0.0,
            Waypoint.fromHolonomicPose(goOutForFirstButShoot.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(-0.5, 1.0)), null, new Rotation2d(Math.PI)),
            // new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(Units.feetToMeters(-3.0), Units.feetToMeters(2.0))), new Rotation2d(), null),
            new Waypoint(FieldConfig.getInstance().NOTE_F.minus(new Translation2d(Units.feetToMeters(0.5), Units.feetToMeters(0.0))), new Rotation2d(), Rotation2d.fromDegrees(225))
        );

        
        
        Rotation2d temp = FieldConfig.getInstance().NOTE_H.minus(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0, 1.625))).getAngle();
        var grabHFirst = SwervePathBuilder.generate(maxVel, maxAcc, 0.675, 0.0,
            Waypoint.fromHolonomicPose(goOutForFirstButShoot.getFinalHolonomicPose()),
            // new Waypoint(temp, FieldConfig.getInstance().NOTE_H.minus(temp).getAngle(), FieldConfig.getInstance().NOTE_H.minus(temp).getAngle()),
            // new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0, 0.3)), null, temp.minus(Rotation2d.fromDegrees(0))),
            new Waypoint(FieldConfig.getInstance().NOTE_H, temp, temp)
        );

        grabFirstPaths.put(CENTERLINE_NOTES.G, grabGFirst);
        grabFirstPaths.put(CENTERLINE_NOTES.F, grabFFirst);
        grabFirstPaths.put(CENTERLINE_NOTES.H, grabHFirst);


        var grabGSecond = SwervePathBuilder.generate(maxVel, maxAcc, 0.675, 0.0,
            Waypoint.fromHolonomicPose(secondShotPose),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0, 1.0)), null, new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().NOTE_G.plus(new Translation2d(Units.feetToMeters(1), -Units.feetToMeters(0.6))), new Rotation2d(Math.PI), null)
        );

        // var grabFSecond = SwervePathBuilder.generate(maxVel, maxAcc,
        //     Waypoint.fromHolonomicPose(secondShotPose),
        //     new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0.5, 1.0)), null, new Rotation2d()),
        //     new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(Units.feetToMeters(-3.0), Units.feetToMeters(-0.5))), new Rotation2d(), new Rotation2d()),
        //     new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(Units.feetToMeters(0.5), Units.feetToMeters(-0.5))), new Rotation2d(), new Rotation2d())
        // );

        var grabFSecond = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(secondShotPose),
            // new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.interpolate(FieldConfig.getInstance().RED_CENTER_STAGE, 0.5), null, Rotation2d.fromDegrees(240)),
            // new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.interpolate(FieldConfig.getInstance().RED_STAGE_LEFT, 0.5), null, new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_LEFT.interpolate(FieldConfig.getInstance().RED_STAGE_RIGHT, 0.5).interpolate(FieldConfig.getInstance().RED_CENTER_STAGE, 0.5), null, new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(Units.feetToMeters(-0.5), Units.feetToMeters(0.5))), new Rotation2d(Math.PI), new Rotation2d(Math.PI))
        );

        var grabHSecond = SwervePathBuilder.generate(maxVel, maxAcc, 0.675, 0.0,
            Waypoint.fromHolonomicPose(secondShotPose),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0, 1.0)), null, new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().NOTE_H.plus(new Translation2d(Units.feetToMeters(-1), -Units.feetToMeters(1.5))))
        );

        grabSecondPaths.put(CENTERLINE_NOTES.G, grabGSecond);
        grabSecondPaths.put(CENTERLINE_NOTES.F, grabFSecond);
        grabSecondPaths.put(CENTERLINE_NOTES.H, grabHSecond);


        var comeBackScoreG = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(grabGFirst.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0, 1.4)), null, new Rotation2d(Math.PI)),
            Waypoint.fromHolonomicPose(secondShotPose.transformBy(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(5))))
        );

        var comeBackScoreF = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(grabFSecond.getFinalHolonomicPose()),
            // new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0.5, 1.0)), null, new Rotation2d()),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.interpolate(FieldConfig.getInstance().RED_STAGE_LEFT, 0.5), null, new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.interpolate(FieldConfig.getInstance().RED_CENTER_STAGE, 0.7), null, new Rotation2d(Math.PI)),
            Waypoint.fromHolonomicPose(secondShotPose.transformBy(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(0)))) // 3.5
        );

        var comeBackScoreH = SwervePathBuilder.generate(maxVel, maxAcc,
            Waypoint.fromHolonomicPose(grabHFirst.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0, 1.0)), null, new Rotation2d(Math.PI)),
            Waypoint.fromHolonomicPose(secondShotPose.transformBy(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(5))))
        );

        scorePaths.put(CENTERLINE_NOTES.G, comeBackScoreG);
        scorePaths.put(CENTERLINE_NOTES.F, comeBackScoreF);
        scorePaths.put(CENTERLINE_NOTES.H, comeBackScoreH);

        // comeBackScoreF = SwervePathBuilder.generate(maxVel, maxAcc,
        //     Waypoint.fromHolonomicPose(goOutForF.getFinalHolonomicPose()),
        //     new Waypoint(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(-0.5, 1.0)), null, new Rotation2d(Math.PI)),
        //     Waypoint.fromHolonomicPose(secondShotPose)
        // );


        this.note1 = note1;
        this.note2 = note2;
        this.note3 = note3;
    }

    @Override
    public CENTERLINE_NOTES[] priorityList() {
        return new CENTERLINE_NOTES[] { note1, note2, note3 };
    }

    @Override
    public Command routine() {
        return Commands.sequence(
            RobotContainer.setArmAndWristDown(), // TODO: accounts for bad pid
            RobotContainer.shooter.rampUp().alongWith(RobotContainer.hintAng(0, 0)),
            Commands.deadline( // grabs first note while shooting P
                Commands.sequence(
                    drive(goOutForFirstButShoot),
                    drive(grabFirstPaths.get(this.note1)).fetchAtEndFor(0.875).withTimeout(grabFirstPaths.get(this.note1).getTotalTimeSeconds() + 0.25)
                ),
                Commands.waitUntil(() -> RobotContainer.drive.getPose().getTranslation().getX() <= firstShotPos.getX() + Units.feetToMeters(1.0)) // 2
                    .deadlineWith(RobotContainer.hintAng(0, 0))
                    .andThen(
                        waitSeconds(0.1).deadlineWith(RobotContainer.wrist.setWristDown()),
                        RobotContainer.shootSequence(),
                        RobotContainer.intake().alongWith(RobotContainer.wrist.intakeAssist())
                    )
            ),
            
            driveAndIntake(scorePaths.get(this.note1)).alongWith(RobotContainer.hintAng(aimMap.get(this.note1))) // 3, 2)
                .deadlineWith(
                    RobotContainer.startAim()
                ),
            waitSeconds(0.1).deadlineWith(RobotContainer.wrist.setWristDown()),
            RobotContainer.shootSequence(),

            driveAndFetchFixed(grabSecondPaths.get(this.note2), 1.0)
            .deadlineWith(RobotContainer.wrist.setWristDown()).withTimeout(grabSecondPaths.get(this.note2).getTotalTimeSeconds()),
            driveAndIntake(scorePaths.get(this.note2))
                .deadlineWith(RobotContainer.hintAng(aimMap.get(this.note2))), // 0.8, 0
            waitSeconds(0.1),
            RobotContainer.shootSequence(),

            driveAndFetchFixed(grabSecondPaths.get(this.note3), 1.0)
            .deadlineWith(RobotContainer.wrist.setWristDown()).withTimeout(grabSecondPaths.get(this.note3).getTotalTimeSeconds()),
            driveAndIntake(scorePaths.get(this.note3))
                .deadlineWith(RobotContainer.hintAng(aimMap.get(this.note3))), // 0.5, 0
            waitSeconds(0.1),
            RobotContainer.shootSequence()
        );
    }

    @Override
    public Route getRoute() {
        return new Route()
            .addPaths(goOutForFirstButShoot, grabFirstPaths.get(this.note1), scorePaths.get(this.note1), grabSecondPaths.get(this.note2), scorePaths.get(this.note2), grabSecondPaths.get(this.note3), scorePaths.get(this.note3));
    }
    
}
