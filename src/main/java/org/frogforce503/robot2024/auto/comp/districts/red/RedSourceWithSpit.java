package org.frogforce503.robot2024.auto.comp.districts.red;

import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import org.frogforce503.lib.auto.AutoMode;
import org.frogforce503.lib.trajectory.PlannedPath;
import org.frogforce503.lib.trajectory.Route;
import org.frogforce503.lib.trajectory.SwervePathBuilder;
import org.frogforce503.lib.trajectory.Waypoint;
import org.frogforce503.robot2024.RobotContainer;
import org.frogforce503.robot2024.auto.AutoUtil.Waypoints;
import org.frogforce503.robot2024.fields.FieldConfig;
import org.frogforce503.robot2024.planners.ShotPlanner.Presets;
import org.frogforce503.robot2024.subsystems.sim.Visualizer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RedSourceWithSpit extends AutoMode {

    private PlannedPath getDAndSpitFirst, grabAndScoreE, grabAndScoreF, grabAndScoreP;
    private double firstSpit;
    private Translation2d NOTE_P;

    public RedSourceWithSpit() {
        Pose2d startingPose = setupPose(
            new Pose2d(
                new Translation2d(FieldConfig.getInstance().RED_INITIATION_LINE - 0.5, FieldConfig.getInstance().NOTE_D.getY()),
                new Rotation2d(Math.PI)
            )
        );

        Pose2d shootingPose = Waypoints.facingGoal(FieldConfig.getInstance().RED_STAGE_LEFT.plus(new Translation2d(1.25, -0.75)));

        getDAndSpitFirst = SwervePathBuilder.generate(5.25, 4.5,
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(new Translation2d(FieldConfig.getInstance().RED_CENTER_STAGE.getX(), FieldConfig.getInstance().NOTE_D.getY()), new Rotation2d(Math.PI), Rotation2d.fromDegrees(-154)),
            new Waypoint(new Translation2d(FieldConfig.getInstance().RED_WING_LINE, FieldConfig.getInstance().NOTE_D.getY()), new Rotation2d(Math.PI), Rotation2d.fromDegrees(-154)),
            new Waypoint(FieldConfig.getInstance().NOTE_D, new Rotation2d(Math.PI), new Rotation2d(Math.PI)),
            Waypoint.fromHolonomicPose(shootingPose)
        );

        firstSpit = FieldConfig.getInstance().RED_WING_LINE + Units.feetToMeters(5);

        grabAndScoreE = SwervePathBuilder.generate(5.25, 4.5,
            Waypoint.fromDifferentialPose(getDAndSpitFirst.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().NOTE_E, null, new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_LEFT.plus(new Translation2d(0.0, -0.75))),
            Waypoint.fromHolonomicPose(shootingPose)
        );

        grabAndScoreF = SwervePathBuilder.generate(5.25, 4.5,
            Waypoint.fromDifferentialPose(grabAndScoreE.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_LEFT.plus(new Translation2d(0.0, -0.75))),
            new Waypoint(FieldConfig.getInstance().NOTE_F, null, Rotation2d.fromDegrees(135)),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_LEFT.plus(new Translation2d(0.0, -0.75))),
            Waypoint.fromHolonomicPose(shootingPose)
        );

        NOTE_P = FieldConfig.getInstance().RED_CENTER_STAGE.plus(new Translation2d(2.0, -1.0));

        grabAndScoreP = SwervePathBuilder.generate(5.25, 4.5,
            Waypoint.fromHolonomicPose(grabAndScoreE.getFinalHolonomicPose()),
            new Waypoint(NOTE_P, NOTE_P.minus(shootingPose.getTranslation()).getAngle(), NOTE_P.minus(shootingPose.getTranslation()).getAngle()),
            Waypoint.fromHolonomicPose(Waypoints.facingGoal(FieldConfig.getInstance().RED_SPEAKER.plus(new Translation2d(-1, -1))).plus(new Transform2d(new Translation2d(), new Rotation2d(Math.PI))))
        );

    }

    @Override
    public Command routine() {
        return Commands.sequence(
            RobotContainer.setToSpit(),
            drive(getDAndSpitFirst)
                .deadlineWith(
                    Commands.sequence(
                        Commands.waitUntil(() -> {
                            boolean c = RobotContainer.drive.getPose().getX() <= firstSpit;
                            System.out.println(c);
                            return c;
                        }),
                        RobotContainer.spitNote().withTimeout(1.0),
                        Commands.runOnce(() -> {
                            if (RobotBase.isSimulation())
                                Visualizer.getInstance().addFloorNotes(NOTE_P);
                        }),
                        RobotContainer.intake().until(RobotContainer.feeder::noteInExitSensor),
                        RobotContainer.startAim()
                    )
                ),
            RobotContainer.shootSequence(),
            driveAndIntake(grabAndScoreE),
            RobotContainer.shootSequence(),
            driveAndIntake(grabAndScoreF),
            RobotContainer.shootSequence(),
            drive(grabAndScoreP)
                .deadlineWith(
                    Commands.sequence(
                        RobotContainer.intake().until(RobotContainer.feeder::noteInExitSensor),
                        RobotContainer.selectPreset(Presets.FRONT_BATTER)
                    )
                ),
            RobotContainer.shootSequence()
        );
    }

    @Override
    public Route getRoute() {
        return new Route(getDAndSpitFirst, grabAndScoreE, grabAndScoreF, grabAndScoreP);
    }
}
