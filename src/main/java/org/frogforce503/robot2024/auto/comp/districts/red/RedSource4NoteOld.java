package org.frogforce503.robot2024.auto.comp.districts.red;

import static edu.wpi.first.wpilibj2.command.Commands.none;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.frogforce503.lib.auto.AutoMode;
import org.frogforce503.lib.trajectory.PlannedPath;
import org.frogforce503.lib.trajectory.Route;
import org.frogforce503.lib.trajectory.SwervePathBuilder;
import org.frogforce503.lib.trajectory.Waypoint;
import org.frogforce503.lib.trajectory.Route.Tree;
import org.frogforce503.lib.util.AllianceFlipUtil;
import org.frogforce503.robot2024.Robot;
import org.frogforce503.robot2024.RobotContainer;
import org.frogforce503.robot2024.auto.AutoUtil.Waypoints;
import org.frogforce503.robot2024.fields.FieldConfig;
import org.frogforce503.robot2024.planners.ShotPlanner;
import org.frogforce503.robot2024.subsystems.NoteDetector.SIDE;
import org.frogforce503.robot2024.subsystems.NoteDetector.SORTING_MODE;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RedSource4NoteOld extends AutoMode{

    // private PlannedPath grabFirstRoot, grabSecondRoot;
    
    // private PlannedPath grabGBranch, grabFBranch, grabEBranch, grabNoneBranch;
    // private PlannedPath[] grabBranches;
    // private Tree grabFirstTree, grabSecondTree, scoreTree;
    
    // private PlannedPath goAroundForE, scoreE, pickUpAndScoreI;

    private PlannedPath grabG, scoreG, grabF, scoreF, pickUpAndScoreI;

    private Waypoint.Tag firstDiversion = new Waypoint.Tag();
    private Waypoint.Tag secondDiversion = new Waypoint.Tag();

    private double maxSpeed = 4.0; // 3.875;
    private double maxAccel = 3.5;

    private double mainShotHint = 0.0;
    private double shotIHint = 0.0;

    public RedSource4NoteOld() {
        RobotContainer.noteDetector.setSortingMode(SORTING_MODE.LEFT_TO_RIGHT);

        Pose2d startingPose = setupPose(
            Waypoints.facingGoal(new Pose2d(
                FieldConfig.getInstance().RED_INITIATION_LINE + Robot.bot.fullRobotLength / 2,
                FieldConfig.getInstance().NOTE_A.getY() - 1,
                new Rotation2d())
            )
        );

        // Pose2d startingPose = new Pose2d(
        //     FieldConfig.getInstance().INITIATION_LINE_X() - Robot.bot.fullRobotLength / 2,
        //     FieldConfig.getInstance().NOTE_SOURCE_SIDE().getY() - Units.inchesToMeters(57),
        //     new Rotation2d()
        // );

        // grabGFirst = SwervePathBuilder.generate(maxSpeed, maxAccel, 
        //     Waypoint.fromHolonomicPose(startingPose),
        //     new Waypoint(FieldConfig.getInstance().CENTERSTATE().interpolate(FieldConfig.getInstance().STAGE_SOURCE_SIDE(), 0.5)),
        //     new Waypoint(FieldConfig.getInstance().STAGE_AMP_SIDE().interpolate(FieldConfig.getInstance().STAGE_SOURCE_SIDE(), 0.45), null, new Rotation2d())
        //         .tagged(firstDiversion),
        //     new Waypoint(FieldConfig.getInstance().NOTE_G.plus(new Translation2d(-Units.feetToMeters(2), 0)), new Rotation2d(), new Rotation2d())
        // );

        // divertGForF = SwervePathBuilder.generate(maxSpeed, maxAccel,
        //     new ArrayList<Waypoint>() {{
        //         addAll(grabGFirst.getWaypoints().subList(0, 3));
        //         add(new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(-Units.feetToMeters(2), 0)), new Rotation2d(), new Rotation2d()));
        //     }}
        // );

        // divertGForE = SwervePathBuilder.generate(maxSpeed, maxAccel,
        //     new ArrayList<Waypoint>() {{
        //         addAll(grabGFirst.getWaypoints().subList(0, 3));
        //         add(new Waypoint(FieldConfig.getInstance().NOTE_E.plus(new Translation2d(-Units.feetToMeters(2), 0)), new Rotation2d(), new Rotation2d()));
        //     }}
        // );

        // divertForNothing = SwervePathBuilder.generate(maxSpeed, maxAccel,
        //     new ArrayList<Waypoint>() {{
        //         addAll(grabGFirst.getWaypoints().subList(0, 3));
        //         add(new Waypoint(FieldConfig.getInstance().STAGE_AMP_SIDE().interpolate(FieldConfig.getInstance().STAGE_SOURCE_SIDE().plus(new Translation2d(Units.feetToMeters(1), 0)), 0.45), null, new Rotation2d()));
        //     }}
        // );

        // grabFirstBranches = new PlannedPath[] { grabGFirst, divertGForF, divertGForE, divertForNothing };

        Translation2d centerToStageLeft = FieldConfig.getInstance().RED_STAGE_LEFT.minus(FieldConfig.getInstance().RED_CENTER_STAGE);

        grabG = SwervePathBuilder.generate(maxSpeed, maxAccel, 
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().RED_CENTER_STAGE.interpolate(FieldConfig.getInstance().RED_STAGE_LEFT, 0.35)
                .plus(new Translation2d(Units.feetToMeters(3), centerToStageLeft.getAngle().plus(new Rotation2d(Math.PI/2)))), centerToStageLeft.getAngle().minus(new Rotation2d(Math.PI/2)), centerToStageLeft.getAngle().minus(Rotation2d.fromDegrees(70))),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_LEFT.interpolate(FieldConfig.getInstance().RED_STAGE_RIGHT, 0.5), new Rotation2d(Math.PI), new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().NOTE_G.minus(new Translation2d(-Units.feetToMeters(0.5), 0.0)).minus(new Translation2d(Units.feetToMeters(1.0), Rotation2d.fromDegrees(160))),Rotation2d.fromDegrees(160), Rotation2d.fromDegrees(160)),
            new Waypoint(FieldConfig.getInstance().NOTE_G.minus(new Translation2d(-Units.feetToMeters(0.5), Units.feetToMeters(-0.5))), new Rotation2d(Math.PI), Rotation2d.fromDegrees(160))
        );


        // var grabEBranch = SwervePathBuilder.generate(maxSpeed, maxAccel, 3, 0,
        //     grabFirstTrunk.getFinalWaypoint(),
        //     new Waypoint(FieldConfig.getInstance().NOTE_E.minus(new Translation2d(-Units.feetToMeters(2), 0)), new Rotation2d(Math.PI), new Rotation2d(Math.PI))
        // );

        // var grabNoneBranch = SwervePathBuilder.generate(maxSpeed, maxAccel, 3, 0,
        //     grabFirstTrunk.getFinalWaypoint(),
        //     new Waypoint(FieldConfig.getInstance().STAGE_AMP_SIDE().interpolate(FieldConfig.getInstance().STAGE_SOURCE_SIDE(), 0.45).plus(new Translation2d(Units.feetToMeters(1), 0)), null, new Rotation2d())
        // );

        // grabFirstTree = new Tree()
        //     .withTrunk(grabFirstTrunk)
        //     .withBranches(grabGBranch, grabFBranch, grabEBranch, grabNoneBranch);

        Pose2d shootingPos = Waypoints.facingGoal(
            new Pose2d(new Translation2d(12.85, 2.75), //FieldConfig. ().NOTE_SOURCE_SIDE().minus(new Translation2d(0, 1)),
            new Rotation2d())
        );

        mainShotHint = ShotPlanner.getInstance().getRawDistanceToGoal(shootingPos.getTranslation());

        scoreG = SwervePathBuilder.generate(maxSpeed + 0.25, maxAccel,
            Waypoint.fromHolonomicPose(grabG.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_LEFT.interpolate(FieldConfig.getInstance().RED_STAGE_RIGHT, 0.5), null, null),
            new Waypoint(FieldConfig.getInstance().RED_CENTER_STAGE.interpolate(FieldConfig.getInstance().RED_STAGE_LEFT, 0.5)),
            Waypoint.fromHolonomicPose(shootingPos)
        );

        grabF = SwervePathBuilder.generate(maxSpeed, maxAccel, 
            new ArrayList<Waypoint>() {{
                add(Waypoint.fromHolonomicPose(shootingPos));
                addAll(grabG.getWaypoints().subList(2, 3));
                add(new Waypoint(FieldConfig.getInstance().NOTE_F.minus(new Translation2d(-Units.feetToMeters(2), 0)), new Rotation2d(Math.PI), new Rotation2d(Math.PI)));
            }}
        );

        scoreF = SwervePathBuilder.generate(maxSpeed + 0.25, maxAccel, 0, 1.5,
            Waypoint.fromHolonomicPose(grabF.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().RED_STAGE_LEFT.interpolate(FieldConfig.getInstance().RED_STAGE_RIGHT, 0.5), null, new Rotation2d(Math.PI)),
            new Waypoint(FieldConfig.getInstance().RED_CENTER_STAGE.interpolate(FieldConfig.getInstance().RED_STAGE_LEFT, 0.5)),
            Waypoint.fromHolonomicPose(shootingPos)
        );;


        Pose2d pointIPickup = Waypoints.facingGoal(FieldConfig.getInstance().NOTE_A.plus(new Translation2d(Units.feetToMeters(1.0), Units.feetToMeters(1.0))));
        
        pickUpAndScoreI = SwervePathBuilder.generate(4.25, 3.75, 1.5, 0,
            Waypoint.fromHolonomicPose(shootingPos),
            Waypoint.fromHolonomicPose(pointIPickup.transformBy(new Transform2d(new Translation2d(Units.feetToMeters(2.0), Units.feetToMeters(3.0)), new Rotation2d()))),
            Waypoint.fromHolonomicPose(pointIPickup)
            // new Waypoint(FieldConfig.getInstance().NOTE_SOURCE_SIDE().minus(new Translation2d(0.5, 0)), null, new Rotation2d())
        );

        shotIHint = ShotPlanner.getInstance().getRawDistanceToGoal(pickUpAndScoreI.getFinalHolonomicPose().getTranslation());
        // comeBackScoreG = SwervePathBuilder.generate(maxSpeed, maxAccel,
        //     Waypoint.fromHolonomicPose(grabGFirst.getFinalHolonomicPose()),
        //     new Waypoint(FieldConfig.getInstance().STAGE_AMP_SIDE().interpolate(FieldConfig.getInstance().STAGE_SOURCE_SIDE(), 0.375), null, new Rotation2d()),
        //     new Waypoint(FieldConfig.getInstance().CENTERSTATE().interpolate(FieldConfig.getInstance().STAGE_SOURCE_SIDE(), 0.5)),
        //     Waypoint.fromHolonomicPose(shootingPos)
        // );

        // grabF = SwervePathBuilder.generate(maxSpeed, maxAccel, 
        //     Waypoint.fromHolonomicPose(comeBackScoreG.getFinalHolonomicPose()),
        //     new Waypoint(FieldConfig.getInstance().CENTERSTATE().interpolate(FieldConfig.getInstance().STAGE_SOURCE_SIDE(), 0.5)),
        //     new Waypoint(FieldConfig.getInstance().STAGE_AMP_SIDE().interpolate(FieldConfig.getInstance().STAGE_SOURCE_SIDE(), 0.5), null, new Rotation2d()),
        //     new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(-Units.feetToMeters(2), 0)), new Rotation2d(), new Rotation2d())
        // );

        // comeBackScoreF = SwervePathBuilder.generate(maxSpeed, maxAccel,
        //     Waypoint.fromHolonomicPose(grabF.getFinalHolonomicPose()),
        //     new Waypoint(FieldConfig.getInstance().STAGE_AMP_SIDE().interpolate(FieldConfig.getInstance().STAGE_SOURCE_SIDE(), 0.5), null, new Rotation2d()),
        //     new Waypoint(FieldConfig.getInstance().CENTERSTATE().interpolate(FieldConfig.getInstance().STAGE_SOURCE_SIDE(), 0.5)),
        //     Waypoint.fromHolonomicPose(shootingPos)
        // );
    }

    public Command routine() {
        // return decide(grabFirstBranches, firstDiversion, () -> {
        //     System.out.println("CHECKING CAM");
        //     return RobotContainer.noteDetector.firstAvailableNote().priorityIndex;
        // });
        return Commands.sequence(
            RobotContainer.shootSequence(),
            driveAndIntake(grabG),
            // decideAndIntake(grabFirstTree, RobotContainer.noteDetector::firstAvailableNote, firstDiversion),
            drive(scoreG).deadlineWith(RobotContainer.intake(), RobotContainer.hint(mainShotHint)),
            RobotContainer.shootSequence(),
            driveAndIntake(grabF),
            drive(scoreF).deadlineWith(RobotContainer.intake(), RobotContainer.hint(mainShotHint - Units.feetToMeters(2))),
            drive(pickUpAndScoreI, () -> true)
                .deadlineWith(
                    Commands.sequence(
                        RobotContainer.shootSequence(), 
                        Commands.parallel(
                            RobotContainer.hint(shotIHint),
                            RobotContainer.intake()
                        )
                    )
                ),
            RobotContainer.shootSequence()
        );
        // return driveAndIntake(grabGFirst)
        //     .beforeStarting(Commands.waitSeconds(1))
        //     .andThen(
        //         drive(comeBackScoreG),
        //         Commands.waitSeconds(1),
        //         driveAndIntake(grabF),
        //         drive(comeBackScoreF),
        //         Commands.waitSeconds(1),
        //         driveAndIntake(pickUpAndScoreI),
        //         Commands.waitSeconds(1)
        //     );
    }

    @Override
    public Route getRoute() {
        return new Route()
            // .addTrees(grabFirstTree, scoreTree, grabSecondTree)
            .addPaths(grabG, scoreG, grabF, scoreF, pickUpAndScoreI);
        // return new PlannedPath[] {grabFirstRoot, grabGBranch, grabFBranch, grabEBranch, grabNoneBranch }; //, comeBackScoreG, grabF, comeBackScoreF, pickUpAndScoreI};
    }
}