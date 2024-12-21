package org.frogforce503.robot2024.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Function;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import org.frogforce503.robot2024.subsystems.sim.Visualizer;
import org.littletonrobotics.junction.Logger;
import org.frogforce503.lib.auto.AutoChooser.CENTERLINE_NOTES;
import org.frogforce503.robot2024.Robot;
import org.frogforce503.robot2024.RobotContainer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NoteDetector extends SubsystemBase {
    private PhotonCamera noteDetCam;
    private PhotonPipelineResult result;
    private List<PhotonTrackedTarget> targets;
    private PhotonTrackedTarget bestTarget;

    private static NoteDetector instance = null;
    NetworkTableInstance inst;
    NetworkTable photonVisionTable, noteDetTable;
    DoubleSubscriber noteDistanceXSub,noteDistanceYSub, noteFieldXSub, noteFieldYSub, pitchSub, yawSub;
    DoublePublisher noteDistanceXPub, noteDistanceYPub, noteFieldXPub, noteFieldYPub, pitchPub, yawPub; 

    private Translation2d lastBestNoteToRobot = new Translation2d();
    private Translation2d lastBestNoteToField = new Translation2d();

    private SORTING_MODE sortingMode = SORTING_MODE.LEFT_TO_RIGHT;

    public static boolean NOTE_DETECTION_DISABLED = false; // TODO: UPDATE THIS IF NEEDED

    private HashMap<CENTERLINE_NOTES, Boolean> simulatedPresence = new HashMap<>();
    private ArrayList<Translation2d> simulatedNotesInView = new ArrayList<>();

    public NoteDetector(){
        noteDetCam = new PhotonCamera("NoteDetCam");
        inst = NetworkTableInstance.getDefault();

        photonVisionTable = inst.getTable("photonvision");
        noteDetTable = photonVisionTable.getSubTable("note_detection_camera");

        noteDistanceXPub = photonVisionTable.getDoubleTopic("noteDistanceX").publish();
        noteDistanceYPub = photonVisionTable.getDoubleTopic("noteDistanceY").publish();
        noteFieldXPub =  photonVisionTable.getDoubleTopic("noteFieldX").publish();
        noteFieldYPub = photonVisionTable.getDoubleTopic("noteFieldY").publish();
        pitchPub = photonVisionTable.getDoubleTopic("pitch").publish();
        yawPub = photonVisionTable.getDoubleTopic("yaw").publish();

        noteDistanceXSub = photonVisionTable.getDoubleTopic("noteDistanceX").subscribe(0);
        noteDistanceYSub = photonVisionTable.getDoubleTopic("noteDistanceY").subscribe(0);
        noteFieldXSub =  photonVisionTable.getDoubleTopic("noteFieldX").subscribe(0);
        noteFieldYSub = photonVisionTable.getDoubleTopic("noteFieldY").subscribe(0);
        yawSub = photonVisionTable.getDoubleTopic("yaw").subscribe(0);
        pitchSub = photonVisionTable.getDoubleTopic("pitch").subscribe(0);

        noteDetCam.setPipelineIndex(0);

        resetSimulatedPresence();
    }

    public static NoteDetector getInstance() {
        if (instance == null) { instance = new NoteDetector(); }
        return instance;
    }
    
    public void updateTargets() {
        result = noteDetCam.getLatestResult();

        if (result.hasTargets()) {
            targets = result.getTargets();
            bestTarget = result.getBestTarget();
        }

        if (RobotBase.isSimulation()) {
            updateSimulatedNotes();
        }
    }

    private void updateSimulatedNotes() {
        ArrayList<Translation2d> notes = new ArrayList<>();
        Pose2d cameraPose = new Pose3d(RobotContainer.drive.getPose()).transformBy(Robot.bot.NOTE_DETECTION_CAMERA_TO_CENTER).toPose2d();

        for (var note : Visualizer.getInstance().getFloorNotes()) {
            Translation2d noteToCamera = note.minus(cameraPose.getTranslation());

            if (noteToCamera.getAngle().minus(cameraPose.getRotation()).getDegrees() <= 45 && noteToCamera.getNorm() <= 2.5) {
                notes.add(note);
            }
        }

        simulatedNotesInView = notes;
    }

    public boolean noteInView() {
        if (RobotBase.isSimulation())
            return !simulatedNotesInView.isEmpty();

        if (this.result == null) {
            return false;
        }
        return this.result.hasTargets();
    }

    public boolean cameraStatus() {
        return noteDetCam.isConnected();
    }

    private SIDE lastSelectedSide = null;

    public SIDE firstAvailableNote() {
        System.out.println("CHECKING CAMERA");

        if (NOTE_DETECTION_DISABLED || !cameraStatus()) {
            boolean leftToRight = this.sortingMode == SORTING_MODE.LEFT_TO_RIGHT;
            if (lastSelectedSide == null) {
                lastSelectedSide = leftToRight ? SIDE.LEFT : SIDE.RIGHT;
                return lastSelectedSide;
            }

            switch (lastSelectedSide) {
                case LEFT:
                    lastSelectedSide = leftToRight ? SIDE.CENTER : SIDE.NONE;
                    break;
                case CENTER:
                    lastSelectedSide = leftToRight ? SIDE.RIGHT : SIDE.LEFT;
                    break;
                case RIGHT:
                    lastSelectedSide = leftToRight ? SIDE.NONE : SIDE.CENTER;
                    break;
                case NONE:
                    lastSelectedSide = SIDE.NONE;
                    break;
            }

            return lastSelectedSide;
        }

        if (this.bestTarget == null) {
            return SIDE.NONE;
        }

        if (this.bestTarget.getYaw() < -10) {
            return SIDE.LEFT;
        } else if (this.bestTarget.getYaw() < 10) {
            return SIDE.CENTER;
        } else {
            return SIDE.RIGHT;
        }
    }

    public void setSortingMode(SORTING_MODE mode) { // should NOT change the pipeline
        if (mode == SORTING_MODE.LEFT_TO_RIGHT) {
            noteDetCam.setPipelineIndex(0);
        } else if (mode == SORTING_MODE.RIGHT_TO_LEFT) {
            noteDetCam.setPipelineIndex(1);
        } else {
            noteDetCam.setPipelineIndex(2);
        }
        this.sortingMode = mode;
    }

    public boolean isNotePresent(SIDE side) {
        System.out.println(side + " ANY NOTES IN VIEW: " + noteInView());
        // System.out.println("NUMBER OF NOTES IN VIEW: " + (targets == null ? 0 : targets.size()));
        if (!noteInView()) {
            return false;
        }
        
        for (var note : targets) {
            System.out.println(side + " " + note.getYaw());
            if (side == SIDE.LEFT && note.getYaw() < -13) {
                return true;
            } else if (side == SIDE.CENTER && note.getYaw() <= 13 && note.getYaw() >= -13){
                return true;
            } else if (side == SIDE.RIGHT && note.getYaw() > 13) {
                return true;
            }
        }

        return false;
    }

    public boolean isNotePresent(CENTERLINE_NOTES note, Function<CENTERLINE_NOTES, SIDE> perspective) {
        if (NOTE_DETECTION_DISABLED || !cameraStatus() || RobotBase.isSimulation()) {
            boolean val = simulatedPresence.get(note);
            simulatedPresence.put(note, false);
            System.out.println("Simulated presence of " + note + " is " + val);
            return val;
        } else if (RobotBase.isSimulation()) {
            for (var simNote : simulatedNotesInView) {
                if (simNote.getDistance(note.position.get()) <= 0.5) {
                    return true;
                }
            }
            return false;
        }

        return isNotePresent(perspective.apply(note));
    }

    public void resetSimulatedPresence() {
        simulatedPresence.clear();
        
        // EDIT THESE VALUES TO TEST OBJECT DETECTION RESPONSES IN SIMULATION
        simulatedPresence.put(CENTERLINE_NOTES.H, true);
        simulatedPresence.put(CENTERLINE_NOTES.G, true);
        simulatedPresence.put(CENTERLINE_NOTES.F, true);
        simulatedPresence.put(CENTERLINE_NOTES.E, true);
        simulatedPresence.put(CENTERLINE_NOTES.D, true);
    }

    private Translation2d calculateNoteToRobot(PhotonTrackedTarget target) {
        double distance = PhotonUtils.calculateDistanceToTargetMeters(
            Robot.bot.NOTE_DETECTION_CAMERA_TO_CENTER.getZ(),
            Units.inchesToMeters(1.75/2),
            Robot.bot.NOTE_DETECTION_CAMERA_TO_CENTER.getRotation().getY(),
            Units.degreesToRadians(target.getPitch())
        );

        Translation2d noteToCamera = PhotonUtils.estimateCameraToTargetTranslation(distance, Rotation2d.fromDegrees(-target.getYaw()));
        Translation2d noteToRobot =  noteToCamera.plus(Robot.bot.NOTE_DETECTION_CAMERA_TO_CENTER.getTranslation().toTranslation2d());
        
        return noteToRobot;
    }

    private Translation2d calculateNoteToField(PhotonTrackedTarget target) {
        Translation2d robotToNote = calculateNoteToRobot(target);
        Translation2d robotToField = RobotContainer.drive.getPose().getTranslation();
        Translation2d noteToField = robotToNote.rotateBy(RobotContainer.drive.getAngle()).plus(robotToField);
        
        return noteToField;
    }

    public Translation2d getBestNoteToRobot() {
        if (RobotBase.isSimulation()) {
            Translation2d robot = RobotContainer.drive.getPose().getTranslation();
            Translation2d bestNote = new Translation2d(0.0, 0.0);
            double bestDistance = Double.MAX_VALUE;
            
            for (var note : simulatedNotesInView) {
                if (note.getDistance(robot) < bestDistance) {
                    bestNote = note;
                    bestDistance = note.getDistance(robot);
                }
            }

            Logger.recordOutput("TargetNote", bestNote);

            return robot.minus(bestNote);
        }
        return this.lastBestNoteToRobot;
    }

    @Override
    public void periodic() {
        updateTargets();

        if (noteInView() && RobotBase.isReal()) {
            this.lastBestNoteToRobot = calculateNoteToRobot(bestTarget);
            this.lastBestNoteToField = calculateNoteToField(bestTarget);

            Logger.recordOutput("Vision/NoteToRobot", new Pose2d(lastBestNoteToRobot, new Rotation2d()));
            Logger.recordOutput("Vision/Note", new Pose2d(lastBestNoteToField, new Rotation2d()));
        }

        Logger.recordOutput("Vision/NoteInView", noteInView());
    }

    public List<Translation2d> getAllNotePositions(){
        List<Translation2d> notes = new ArrayList<>();

        for (PhotonTrackedTarget target : targets) {
            notes.add(new Translation2d(target.getYaw(), target.getPitch()));
        }

        return notes;
    }

    public enum SIDE {
        LEFT, CENTER, RIGHT, NONE;

        public int priorityIndex() {
            switch (this) {
                case LEFT:
                    return 0;
                case CENTER:
                    return 1;
                case RIGHT:
                    return 2;
                case NONE:
                    return 3;
            }
            return 3;
        }
    }

    public enum SORTING_MODE {
        LEFT_TO_RIGHT, RIGHT_TO_LEFT, CENTERMOST;
    }
}