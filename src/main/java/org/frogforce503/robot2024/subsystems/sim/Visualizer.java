package org.frogforce503.robot2024.subsystems.sim;

import java.util.ArrayList;
import java.util.Set;

import org.frogforce503.robot2024.RobotContainer;
import org.frogforce503.robot2024.fields.FieldConfig;
import org.frogforce503.robot2024.planners.ShotPlanner;
import org.frogforce503.robot2024.subsystems.Intake.Goal;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;

public class Visualizer {

    private static Visualizer instance;

    public static Visualizer getInstance() {
        if (instance == null) { instance = new Visualizer(); }
        return instance;
    }

    private final double armLength = Units.inchesToMeters(16.509);
    private final Rotation3d armDownAngle = new Rotation3d(0.0, Units.degreesToRadians(21), 0);
    private final Translation3d armPosition = new Translation3d(-0.229, 0, 0.491);
    private final Rotation3d wristDownAngle = new Rotation3d(0.0, -Units.degreesToRadians(167.5), 0.0);

    private final double shotRPMToSpeed = 13.0/4100;

    private Pose3d heldNotePose = new Pose3d();

    private ArrayList<Translation2d> floorNotes = new ArrayList<Translation2d>();

    public void update() {
        double armAngle = RobotContainer.arm.getPosition();
        var armRotation = new Rotation3d(0.0, -Units.degreesToRadians(armAngle), 0.0).plus(armDownAngle);

        var armPose = new Pose3d(armPosition, armRotation);

        var wristAngle = RobotContainer.wrist.getPosition();
        var wristRotation = new Rotation3d(0.0, Units.degreesToRadians(wristAngle), 0.0).plus(wristDownAngle);
        var wristPose = armPose.transformBy(new Transform3d(new Translation3d(armLength, 0, 0), wristRotation));

        Logger.recordOutput("Mechanism", armPose, wristPose);

        boolean gotNote = RobotContainer.feeder.noteInExitSensor();
        if (gotNote) {
            var drivePose = new Pose3d(RobotContainer.drive.getPose());
            heldNotePose = drivePose.transformBy(new Transform3d(wristPose.getTranslation(), wristPose.getRotation()));
            Logger.recordOutput("HeldNote", new Pose3d[] { heldNotePose });
        } else {
            Logger.recordOutput("HeldNote", new Pose3d[] {});
        }

        if (RobotBase.isSimulation() && !gotNote && RobotContainer.intake.goal == Goal.INTAKE) {
            var robot = RobotContainer.drive.getPose();
            int index = 0;
            for (var note : floorNotes) {
                Translation2d displacement = note.minus(robot.getTranslation());
                // check if angle of robot is within 45 degrees of the note

                if (robot.getTranslation().getDistance(note) < Units.inchesToMeters(20) && Math.abs(displacement.getAngle().minus(robot.getRotation()).getRadians()) < Units.degreesToRadians(90)) {
                    RobotContainer.feeder.setSimulatedNotePresent(true);
                    floorNotes.remove(index);
                    break;
                }
                index++;
            }
        }

        Logger.recordOutput("FloorNotes", floorNotes.stream().map(n -> new Pose3d(new Translation3d(n.getX(), n.getY(), Units.inchesToMeters(2)), new Rotation3d())).toArray(Pose3d[]::new));
    }

    public Command visualizeShotNote() {
        return new ScheduleCommand(
            Commands.defer(() -> {
                if (!RobotContainer.feeder.noteInExitSensor())
                    return Commands.none();

                double shotSpeed = RobotContainer.shooter.currentGoal.setpoint.get().output * shotRPMToSpeed;

                RobotContainer.feeder.setSimulatedNotePresent(false);
                Translation3d shotVelocity = new Translation3d(shotSpeed, heldNotePose.getRotation());

                // var bv = RobotContainer.drive.getVelocity().getTranslation();
                // Translation3d vi = shotVelocity;//.plus(new Translation3d(bv.getX(), bv.getY(), 0));
                // x = vi * t + 0.5 * a * t^2

                double shotDuration = ShotPlanner.getInstance().getDistanceTarget() / shotSpeed + 1.0;
                // double gravityEffect = 0.5 * 9.8 * shotDuration * shotDuration * 0.25; // 0.25 is a fudge factor

                // Translation3d endPosition = heldNotePose.getTranslation().plus(shotVelocity.times(shotDuration )).plus(new Translation3d(0, 0, -gravityEffect));
                // Pose3d endPose = new Pose3d(endPosition, heldNotePose.getRotation());

                Timer shotTimer = new Timer();
                shotTimer.start();

                return Commands.run(
                    () -> {
                        var ds = shotVelocity.times(shotTimer.get()).plus(new Translation3d(0, 0, -0.5 * 9.8 * shotTimer.get() * shotTimer.get()));
                        var currentPosition = heldNotePose.getTranslation().plus(ds);
                        Logger.recordOutput("ShotNote", new Pose3d[] {
                            new Pose3d(currentPosition, heldNotePose.getRotation()/* .interpolate(new Rotation3d(0, 0, 0), shotTimer.get() / shotDuration)*/)
                        });
                    }
                ).until(() -> shotTimer.hasElapsed(shotDuration + 0.75))
                .finallyDo(() -> Logger.recordOutput("ShotNote", new Pose3d[] { }));
            }, Set.of())
        ).ignoringDisable(true);
    }

    public Command ampShot() {
        return new ScheduleCommand(
            Commands.defer(() -> {
                if (!RobotContainer.feeder.noteInExitSensor())
                    return Commands.none();

                RobotContainer.feeder.setSimulatedNotePresent(false);

                Translation3d endPosition = new Translation3d(heldNotePose.getX(), heldNotePose.getY(), 0.1)
                    .plus(new Translation3d(Units.inchesToMeters(-20), new Rotation3d(0, 0, RobotContainer.drive.getAngle().getRadians())));
                
                Pose3d endPose = new Pose3d(endPosition, new Rotation3d(0, Math.PI/2, RobotContainer.drive.getAngle().getRadians()));

                Timer shotTimer = new Timer();
                shotTimer.start();

                double shotDuration = 2.0;

                return Commands.run(
                    () -> {
                        Logger.recordOutput("ShotNote", new Pose3d[] {
                            heldNotePose.interpolate(endPose, shotTimer.get() / shotDuration)
                        });
                    }
                ).until(() -> shotTimer.hasElapsed(shotDuration + 0.75))
                .finallyDo(() -> Logger.recordOutput("ShotNote", new Pose3d[] { }));
            }, Set.of())
        ).ignoringDisable(true);
    }

    public void addFloorNotes(Translation2d ...note) {
        for (var n : note) {
            floorNotes.add(n);
        }
    }

    public void resetFloorNotes() {
        floorNotes.clear();
    }

    public void setupAuto() {
        RobotContainer.feeder.setSimulatedNotePresent(true);
        resetFloorNotes();
        addFloorNotes(
            FieldConfig.getInstance().NOTE_A,
            FieldConfig.getInstance().NOTE_B,
            FieldConfig.getInstance().NOTE_C,
            FieldConfig.getInstance().NOTE_D,
            FieldConfig.getInstance().NOTE_E,
            FieldConfig.getInstance().NOTE_F,
            FieldConfig.getInstance().NOTE_G,
            FieldConfig.getInstance().NOTE_H,
            FieldConfig.getInstance().NOTE_I,
            FieldConfig.getInstance().NOTE_J,
            FieldConfig.getInstance().NOTE_K
        );
    }

    public ArrayList<Translation2d> getFloorNotes() {
        return floorNotes;
    }
}
