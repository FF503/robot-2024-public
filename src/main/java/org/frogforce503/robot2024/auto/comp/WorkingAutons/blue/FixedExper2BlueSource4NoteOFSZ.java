/*
 * Shoots preload at batter then picks up and shoots the 3 centerline notes farthest to our right
 * Robot doesn't shoot if it doesn't intake a note and instead goes for the next priority
 * Robot is aware of which centerline note it tries to intake after it uses fetch and uses the correct hints and paths
 */


package org.frogforce503.robot2024.auto.comp.WorkingAutons.blue;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashMap;

import org.frogforce503.lib.auto.AutoMode;
import org.frogforce503.lib.auto.AutoChooser.CENTERLINE_NOTES;
import org.frogforce503.lib.trajectory.PlannedPath;
import org.frogforce503.lib.trajectory.Route;
import org.frogforce503.lib.trajectory.SwervePathBuilder;
import org.frogforce503.lib.trajectory.Waypoint;
import org.frogforce503.robot2024.RobotContainer;
import org.frogforce503.robot2024.auto.AutoUtil.Waypoints;
import org.frogforce503.robot2024.fields.FieldConfig;
import org.frogforce503.robot2024.subsystems.NoteDetector.SIDE;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class FixedExper2BlueSource4NoteOFSZ extends AutoMode {

    private HashMap<CENTERLINE_NOTES, PlannedPath> grabFirstPath = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> grabSecondPath = new HashMap<>();

    private HashMap<CENTERLINE_NOTES, HashMap<CENTERLINE_NOTES, PlannedPath>> corrections = new HashMap<>();

    private HashMap<CENTERLINE_NOTES, PlannedPath> correctFromD = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> correctFromE = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, PlannedPath> correctFromF = new HashMap<>();

    private HashMap<CENTERLINE_NOTES, PlannedPath> scorePath = new HashMap<>();
    private HashMap<CENTERLINE_NOTES, Pair<Double, Double>> aimMap = new HashMap<>();

    private CENTERLINE_NOTES note1, note2, note3;

    private Pose2d shootingPose;

    // private double bailTime = 0.475;

    //11.41, 5.9
    Waypoint.Tag tag;

    public FixedExper2BlueSource4NoteOFSZ(CENTERLINE_NOTES note1, CENTERLINE_NOTES note2, CENTERLINE_NOTES note3) {

        var startingPose = setupPose(Waypoints.facingGoal(new Translation2d(FieldConfig.getInstance().BLUE_INITIATION_LINE - 0.5, FieldConfig.getInstance().NOTE_I.getY())));

        var shootingPositionX = FieldConfig.getInstance().BLUE_CENTER_STAGE.interpolate(FieldConfig.getInstance().BLUE_STAGE_RIGHT, 0.5).getX() - 0.5;
        var shootingPosition = new Translation2d(shootingPositionX - Units.feetToMeters(1), FieldConfig.getInstance().BLUE_SPEAKER.getY() - Units.feetToMeters(1.5));

        shootingPose = Waypoints.facingGoal(shootingPosition);
        // 2.5 arm, 0

        // Arm might be around 0.875-1 for working PID wrist
        aimMap.put(CENTERLINE_NOTES.D, Pair.of(0.0, 2.0)); // prev arm = 2.0
        aimMap.put(CENTERLINE_NOTES.E, Pair.of(0.0, 2.0));
        aimMap.put(CENTERLINE_NOTES.F, Pair.of(0.0, 2.0)); // prev arm = 2.0


        Rotation2d noteDToStart = FieldConfig.getInstance().NOTE_D.minus(startingPose.getTranslation()).getAngle();
        var grabDFirst = SwervePathBuilder.generate(5.0, 4.75, // 5.0, 4.75
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_RIGHT.minus(new Translation2d(0, 1.25)), null, noteDToStart),
            new Waypoint(FieldConfig.getInstance().NOTE_D.plus(new Translation2d(Units.feetToMeters(0.0), Units.inchesToMeters(0))), noteDToStart, noteDToStart)
        );

        var grabEFirst = SwervePathBuilder.generate(4.8, 4.75,

            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_RIGHT.minus(new Translation2d(0, 1.1)), null, Rotation2d.fromDegrees(6)),
            new Waypoint(FieldConfig.getInstance().NOTE_E.plus(new Translation2d(Units.feetToMeters(0.0), Units.inchesToMeters(0))), new Rotation2d(), Rotation2d.fromDegrees(6))
        );

        var grabFFirst = SwervePathBuilder.generate(4.8, 4.75,
            Waypoint.fromHolonomicPose(startingPose),
            new Waypoint(FieldConfig.getInstance().BLUE_CENTER_STAGE.interpolate(FieldConfig.getInstance().BLUE_STAGE_RIGHT, 0.5), Rotation2d.fromDegrees(25), null),
            new Waypoint(FieldConfig.getInstance().NOTE_F.plus(new Translation2d(Units.feetToMeters(0.0), Units.inchesToMeters(0))), new Rotation2d(), new Rotation2d())
        );

        grabFirstPath.put(CENTERLINE_NOTES.D, grabDFirst);
        grabFirstPath.put(CENTERLINE_NOTES.E, grabEFirst);
        grabFirstPath.put(CENTERLINE_NOTES.F, grabFFirst);

        tag = new Waypoint.Tag();


        var scoreD = SwervePathBuilder.generate(4.8, 4.75,
            Waypoint.fromHolonomicPose(grabDFirst.getFinalHolonomicPose()).withHolonomicRotation(Rotation2d.fromDegrees(-30)),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_RIGHT.plus(new Translation2d(1.2, 0.25)), null, shootingPose.getRotation()),
            Waypoint.fromHolonomicPose(shootingPose)
        );


        var scoreE = SwervePathBuilder.generate(4.8, 4.75,
            Waypoint.fromHolonomicPose(grabEFirst.getFinalHolonomicPose()).withHolonomicRotation(Rotation2d.fromDegrees(-30)), //-30
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_LEFT.interpolate(FieldConfig.getInstance().BLUE_STAGE_RIGHT, 0.5), null, shootingPose.getRotation()),
            Waypoint.fromHolonomicPose(shootingPose)
        );
        
        var scoreF = SwervePathBuilder.generate(4.8, 4.75, 0.0, 0,
            Waypoint.fromHolonomicPose(grabFFirst.getFinalHolonomicPose()),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_LEFT.interpolate(FieldConfig.getInstance().BLUE_STAGE_RIGHT, 0.5), new Rotation2d(Math.PI), null),
            Waypoint.fromHolonomicPose(shootingPose)
            
        );

        scorePath.put(CENTERLINE_NOTES.D, scoreD);
        scorePath.put(CENTERLINE_NOTES.E, scoreE);
        scorePath.put(CENTERLINE_NOTES.F, scoreF);

        var grabESecond = SwervePathBuilder.generate(4.8, 4.75,
            Waypoint.fromHolonomicPose(shootingPose),
            new Waypoint(FieldConfig.getInstance().BLUE_STAGE_LEFT.interpolate(FieldConfig.getInstance().BLUE_STAGE_RIGHT, 0.5), null, Rotation2d.fromDegrees(-30)),
            Waypoint.fromHolonomicPose(grabEFirst.getFinalHolonomicPose()).withHolonomicRotation(Rotation2d.fromDegrees(-30))
        );

        grabSecondPath.put(CENTERLINE_NOTES.D, SwervePathBuilder.reversedOf(scoreD, 5.0, 4.75));
        grabSecondPath.put(CENTERLINE_NOTES.E, grabESecond);
        grabSecondPath.put(CENTERLINE_NOTES.F, SwervePathBuilder.reversedOf(scoreF, 5.0, 4.75));

        var correctDToE = SwervePathBuilder.generate(4.0, 3.5,
        Waypoint.fromHolonomicPose(grabDFirst.getFinalHolonomicPose()), 
        new Waypoint(FieldConfig.getInstance().NOTE_D.interpolate(FieldConfig.getInstance().NOTE_E, 0.5)).plus(new Translation2d(-0.5, 0)),
        Waypoint.fromHolonomicPose(grabEFirst.getFinalHolonomicPose())
        );

        var correctDToF = SwervePathBuilder.generate(4.0, 3.5,
        Waypoint.fromHolonomicPose(grabDFirst.getFinalHolonomicPose()), 
        new Waypoint(FieldConfig.getInstance().NOTE_D.interpolate(FieldConfig.getInstance().NOTE_F, 0.5)).plus(new Translation2d(-1.0, 0)),
        Waypoint.fromHolonomicPose(grabFFirst.getFinalHolonomicPose())
        );

        correctFromD.put(CENTERLINE_NOTES.E, correctDToE);
        correctFromD.put(CENTERLINE_NOTES.F, correctDToF);

        corrections.put(CENTERLINE_NOTES.D,correctFromD);

        var correctEToD = SwervePathBuilder.generate(4.0, 3.5,
        Waypoint.fromHolonomicPose(grabEFirst.getFinalHolonomicPose()), 
        new Waypoint(FieldConfig.getInstance().NOTE_E.interpolate(FieldConfig.getInstance().NOTE_D, 0.5)).plus(new Translation2d(-0.5, 0)),
        Waypoint.fromHolonomicPose(grabDFirst.getFinalHolonomicPose())
        );

        var correctEToF = SwervePathBuilder.generate(4.0, 3.5,
        Waypoint.fromHolonomicPose(grabEFirst.getFinalHolonomicPose()), 
        new Waypoint(FieldConfig.getInstance().NOTE_E.interpolate(FieldConfig.getInstance().NOTE_F, 0.5)).plus(new Translation2d(-0.5, 0)),
        Waypoint.fromHolonomicPose(grabFFirst.getFinalHolonomicPose())
        );

        correctFromE.put(CENTERLINE_NOTES.D, correctEToD);
        correctFromE.put(CENTERLINE_NOTES.F, correctEToF);

        corrections.put(CENTERLINE_NOTES.E,correctFromE);

        var correctFToD = SwervePathBuilder.generate(4.0, 3.5,
        Waypoint.fromHolonomicPose(grabFFirst.getFinalHolonomicPose()), 
        new Waypoint(FieldConfig.getInstance().NOTE_F.interpolate(FieldConfig.getInstance().NOTE_D, 0.5)).plus(new Translation2d(-1.0, 0)),
        Waypoint.fromHolonomicPose(grabDFirst.getFinalHolonomicPose())
        );

        var correctFToE = SwervePathBuilder.generate(4.0, 3.5,
        Waypoint.fromHolonomicPose(grabFFirst.getFinalHolonomicPose()), 
        new Waypoint(FieldConfig.getInstance().NOTE_F.interpolate(FieldConfig.getInstance().NOTE_E, 0.5)).plus(new Translation2d(-0.5, 0)),
        Waypoint.fromHolonomicPose(grabEFirst.getFinalHolonomicPose())
        );

        correctFromF.put(CENTERLINE_NOTES.D, correctFToD);
        correctFromF.put(CENTERLINE_NOTES.E, correctFToE);

        corrections.put(CENTERLINE_NOTES.F,correctFromF);
  
        this.note1 = note1;
        this.note2 = note2;
        this.note3 = note3;
    }

    @Override
    public CENTERLINE_NOTES[] priorityList() {
        return new CENTERLINE_NOTES[] {note1, note2, note3};
    }

    int[] storage = {0, 0};
    int[] storage2 = {0, 0};

    @Override
    public Command routine() {
        return Commands.sequence(
            RobotContainer.hintAng(0.0, 12.5),
            RobotContainer.shootSequence(), // TODO Change time out

            RobotContainer.hintAng(aimMap.get(this.note1)), // 0, 0
            driveAndFetchFixed(grabFirstPath.get(this.note1), 0.75).withTimeout(grabFirstPath.get(this.note1).getTotalTimeSeconds() + 0.5),

            Commands.either(                
                Commands.sequence(
                    driveAndIntake(scorePath.get(this.note1)),
                    RobotContainer.shootSequence().beforeStarting(waitSeconds(0.125)),

                    RobotContainer.hintAng(aimMap.get(this.note2)),
                    driveAndFetchFixed(grabSecondPath.get(this.note2), 0.6).withTimeout(grabSecondPath.get(this.note2).getTotalTimeSeconds() + 0.5) 
                    ), 
                Commands.either(
                    Commands.sequence(
                        RobotContainer.hintAng(aimMap.get(this.note2)),
                        driveAndIntake(corrections.get(this.note1).get(this.note2))
                    ),
                    Commands.either(
                        Commands.sequence(
                            RobotContainer.hintAng(aimMap.get(this.note2)),
                            driveAndIntake(scorePath.get(this.note2)),
                            RobotContainer.shootSequence().beforeStarting(waitSeconds(0.125)),
    
                            RobotContainer.hintAng(aimMap.get(this.note3)),
                            driveAndFetchFixed(grabSecondPath.get(this.note3), 0.6).withTimeout(grabSecondPath.get(this.note3).getTotalTimeSeconds() + 0.5)
                            ),
                        Commands.sequence(
                            RobotContainer.hintAng(aimMap.get(this.note3)),
                            driveAndIntake(scorePath.get(this.note3)),
                            RobotContainer.shootSequence().beforeStarting(waitSeconds(0.125)),
        
                            RobotContainer.hintAng(aimMap.get(this.note2)),
                            driveAndFetchFixed(grabSecondPath.get(this.note2), 0.6).withTimeout(grabSecondPath.get(this.note2).getTotalTimeSeconds() + 0.5)
                        ),
                        () -> atCenterLineNote(this.note2)
                    ),
                    () -> atCenterLineNote(note1)
                ),
                () -> atCenterLineNote(note1) && RobotContainer.feeder.noteInEntrySensor()
            ),
         
            Commands.either(                  
                Commands.sequence(
                    driveAndIntake(scorePath.get(note2)),
                    RobotContainer.shootSequence().beforeStarting(waitSeconds(0.125)),
       
                    RobotContainer.hintAng(aimMap.get(this.note3)), // 0, 0
                    driveAndFetchFixed(grabSecondPath.get(note3), 1.0).withTimeout(grabSecondPath.get(note3).getTotalTimeSeconds())      
                ), 
                Commands.either(
                    Commands.sequence(
                        RobotContainer.hintAng(aimMap.get(this.note3)),
                        driveAndIntake(corrections.get(note2).get(note3))
                    ),
                    Commands.sequence(
                        RobotContainer.hintAng(aimMap.get(this.note3)),
                        driveAndIntake(scorePath.get(this.note3)),
                        RobotContainer.shootSequence().beforeStarting(waitSeconds(0.125)),
           
                        RobotContainer.hintAng(aimMap.get(this.note1)), // 0, 0
                        driveAndFetchFixed(grabSecondPath.get(note1), 1.0).withTimeout(grabSecondPath.get(note1).getTotalTimeSeconds())
                    ),
                    () -> atCenterLineNote(this.note2)
                ), 
                () -> atCenterLineNote(this.note2) && RobotContainer.feeder.noteInEntrySensor()
            ),
            Commands.either(
                Commands.sequence(
                    driveAndIntake(scorePath.get(this.note3)),
                    RobotContainer.shootSequence().beforeStarting(waitSeconds(0.125))
                ),
                Commands.sequence(
                    driveAndIntake(scorePath.get(this.note1)),
                    RobotContainer.shootSequence().beforeStarting(waitSeconds(0.125))
                ),
                () -> atCenterLineNote(this.note3)

            )
        );
    }

    private SIDE perspective(CENTERLINE_NOTES note) {
        return (note == CENTERLINE_NOTES.D) ? SIDE.CENTER : (note == CENTERLINE_NOTES.E ? SIDE.RIGHT : SIDE.RIGHT);
    }

    private CENTERLINE_NOTES getCurrentCenterlineNote(){
        CENTERLINE_NOTES currentNote = null;
        double currentRobotY = RobotContainer.drive.getPose().getTranslation().getY();

        if(FieldConfig.getInstance().NOTE_D.getY() - Units.feetToMeters(1.5) < currentRobotY && currentRobotY < FieldConfig.getInstance().NOTE_D.getY() + Units.feetToMeters(1.5)){
            currentNote = CENTERLINE_NOTES.D;
        }else if(FieldConfig.getInstance().NOTE_E.getY() - Units.feetToMeters(1.5) < currentRobotY && currentRobotY < FieldConfig.getInstance().NOTE_E.getY() + Units.feetToMeters(1.5)){
            currentNote = CENTERLINE_NOTES.E;
        }else if(FieldConfig.getInstance().NOTE_F.getY() - Units.feetToMeters(1.5) < currentRobotY && currentRobotY < FieldConfig.getInstance().NOTE_F.getY() + Units.feetToMeters(1.5)){
            currentNote = CENTERLINE_NOTES.F;            
        }
        return currentNote;
    }

    private boolean atCenterLineNote(CENTERLINE_NOTES note){
        if(getCurrentCenterlineNote() == note){
            return true;
        }else{
            return false;
        }
    }

    @Override
    public Route getRoute() {
        return new Route()
            .addPathOptions(grabFirstPath)
            .addPathOptions(grabSecondPath);
    }
    
}
