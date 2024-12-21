package org.frogforce503.lib.auto;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;

import org.frogforce503.lib.drawing.DrawOnField;
import org.frogforce503.lib.trajectory.PlannedPath;
import org.frogforce503.lib.util.SwitchableChooser;
import org.frogforce503.robot2024.RobotContainer;
import org.frogforce503.robot2024.RobotStatus;
import org.frogforce503.robot2024.RobotStatus.AllianceColor;
import org.frogforce503.robot2024.auto.comp.WorkingAutons.blue.BlueAmp4CenterLineOFSZ;
import org.frogforce503.robot2024.auto.comp.WorkingAutons.blue.BlueAmp4CenterLineOFSZ_SHIFT;
import org.frogforce503.robot2024.auto.comp.WorkingAutons.blue.BlueAmp4CenterLineOFSZ_SHIFT_WINGLINE;
import org.frogforce503.robot2024.auto.comp.WorkingAutons.blue.BlueCenter4OFSZ;
import org.frogforce503.robot2024.auto.comp.WorkingAutons.red.RedCenter4OFSZ;
import org.frogforce503.robot2024.auto.comp.WorkingAutons.blue.BlueCleanup5OFSZ;
import org.frogforce503.robot2024.auto.comp.WorkingAutons.blue.AltBlueCleanup5NoteOFSZ;
import org.frogforce503.robot2024.auto.comp.WorkingAutons.blue.BlueSource4NoteOFSZ_SHIFT;
import org.frogforce503.robot2024.auto.comp.WorkingAutons.red.AltRedCleanup5NoteOFSZ;
import org.frogforce503.robot2024.auto.comp.WorkingAutons.red.RedAmp4CenterLineOFSZ;
import org.frogforce503.robot2024.auto.comp.WorkingAutons.red.RedAmp4CenterLineOFSZ_SHIFT;
import org.frogforce503.robot2024.auto.comp.WorkingAutons.red.RedAmp4CenterLineOFSZ_SHIFT_WINGLINE;
import org.frogforce503.robot2024.auto.comp.WorkingAutons.red.RedCleanup5OFSZ;
import org.frogforce503.robot2024.auto.comp.WorkingAutons.red.RedSource4NoteOFSZ;
import org.frogforce503.robot2024.auto.comp.WorkingAutons.red.RedSource4NoteOFSZ_SHIFT;
import org.frogforce503.robot2024.fields.FieldConfig;
import org.frogforce503.robot2024.subsystems.drive.Drive;
import org.frogforce503.robot2024.subsystems.sim.Visualizer;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardString;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoChooser {
    // private HashMap<StartingLocation, HashMap<Integer, AutoMode>> AUTO_MAP = new HashMap<>();

    // GenericEntry autonInfo;

    private LoggedDashboardChooser<StartingLocation> startingSideSelector;
    private LoggedDashboardChooser<AllianceColor> colorSelector;
    private SwitchableChooser<String> routineChooser;
    private LoggedDashboardBoolean commitAuton;
    private LoggedDashboardString selectedAutoNameDisplay;
    private LoggedDashboardBoolean autoReadyDisplay, frontLeftCameraStatus, backLeftCameraStatus, noteDetectorCameraStatus;
    private SwitchableChooser<CENTERLINE_NOTES> priority1Chooser, priority2Chooser, priority3Chooser;

    AutoMode selectedAuto;
    Command selectedAutoCommand;
    Runnable onReset;
    String selectedAutoName = "NO AUTO SELECTED";

    AllianceColor lastAllianceColor = null;
    StartingLocation lastStartingSide = null;
    String lastRoutine = "";
    CENTERLINE_NOTES[] lastPriorityOptions = {null, null, null};

    private static final CENTERLINE_NOTES[] empty = {};

    private static HashMap<AllianceColor, HashMap<StartingLocation, HashMap<String, Pair<CENTERLINE_NOTES[], Supplier<AutoMode>>>>> AUTO_MAP = new HashMap<>();
    static {
        AUTO_MAP.put(AllianceColor.RED,
            new HashMap<StartingLocation, HashMap<String, Pair<CENTERLINE_NOTES[], Supplier<AutoMode>>>>() {{
                put(StartingLocation.LEFT, new HashMap<String, Pair<CENTERLINE_NOTES[], Supplier<AutoMode>>>() {{

                    put("RED_SOURCE_4_CL3_SHIFT", Pair.of(new CENTERLINE_NOTES[] { CENTERLINE_NOTES.D, CENTERLINE_NOTES.E, CENTERLINE_NOTES.F}, 
                    () -> new RedSource4NoteOFSZ_SHIFT(
                        RobotContainer.autoChooser.priority1Chooser.get(), 
                        RobotContainer.autoChooser.priority2Chooser.get(),
                        RobotContainer.autoChooser.priority3Chooser.get()
                    )));

                    // put("RED_SOURCE_4_CL3", Pair.of(new CENTERLINE_NOTES[] { CENTERLINE_NOTES.D, CENTERLINE_NOTES.E, CENTERLINE_NOTES.F}, 
                    // () -> new RedSource4NoteOFSZ(
                    //     RobotContainer.autoChooser.priority1Chooser.get(), 
                    //     RobotContainer.autoChooser.priority2Chooser.get(),
                    //     RobotContainer.autoChooser.priority3Chooser.get()
                    // )));

                }});
                put(StartingLocation.CENTER, new HashMap<String, Pair<CENTERLINE_NOTES[], Supplier<AutoMode>>>() {{

                    put("RED_CENTER_CLEANUP_5", Pair.of(empty, RedCleanup5OFSZ::new));

                    put("ALT_RED_CENTER_CLEANUP_5", Pair.of(empty, AltRedCleanup5NoteOFSZ::new));

                    put("RED_CENTER_PB-CL2", Pair.of(new CENTERLINE_NOTES[] { CENTERLINE_NOTES.E, CENTERLINE_NOTES.F, CENTERLINE_NOTES.G }, 
                    () -> new RedCenter4OFSZ(
                        RobotContainer.autoChooser.priority1Chooser.get(), 
                        RobotContainer.autoChooser.priority2Chooser.get()
                    )));

                }});
                put(StartingLocation.RIGHT, new HashMap<String, Pair<CENTERLINE_NOTES[], Supplier<AutoMode>>>() {{

                    put("RED_AMP_4_CL3_SHIFT_WINGLINE", Pair.of(new CENTERLINE_NOTES[] { CENTERLINE_NOTES.F, CENTERLINE_NOTES.G, CENTERLINE_NOTES.H}, 
                    () -> new RedAmp4CenterLineOFSZ_SHIFT_WINGLINE(
                        RobotContainer.autoChooser.priority1Chooser.get(), 
                        RobotContainer.autoChooser.priority2Chooser.get(),
                        RobotContainer.autoChooser.priority3Chooser.get()
                    )));

                    // put("RED_AMP_4_CL3_SHIFT", Pair.of(new CENTERLINE_NOTES[] { CENTERLINE_NOTES.F, CENTERLINE_NOTES.G, CENTERLINE_NOTES.H}, 
                    // () -> new RedAmp4CenterLineOFSZ_SHIFT(
                    //     RobotContainer.autoChooser.priority1Chooser.get(), 
                    //     RobotContainer.autoChooser.priority2Chooser.get(),
                    //     RobotContainer.autoChooser.priority3Chooser.get()
                    // )));

                    // put("RED_AMP_4_CL3", Pair.of(new CENTERLINE_NOTES[] { CENTERLINE_NOTES.F, CENTERLINE_NOTES.G, CENTERLINE_NOTES.H}, 
                    // () -> new RedAmp4CenterLineOFSZ(
                    //     RobotContainer.autoChooser.priority1Chooser.get(), 
                    //     RobotContainer.autoChooser.priority2Chooser.get(),
                    //     RobotContainer.autoChooser.priority3Chooser.get()
                    // )));
                    
                }});
            }}
        );

        AUTO_MAP.put(AllianceColor.BLUE,
            new HashMap<StartingLocation, HashMap<String, Pair<CENTERLINE_NOTES[], Supplier<AutoMode>>>>() {{
                put(StartingLocation.LEFT, new HashMap<String, Pair<CENTERLINE_NOTES[], Supplier<AutoMode>>>() {{

                    put("BLUE_AMP_4_CL3_SHIFT_WINGLINE", Pair.of(new CENTERLINE_NOTES[] { CENTERLINE_NOTES.F, CENTERLINE_NOTES.G, CENTERLINE_NOTES.H}, 
                    () -> new BlueAmp4CenterLineOFSZ_SHIFT_WINGLINE(
                        RobotContainer.autoChooser.priority1Chooser.get(), 
                        RobotContainer.autoChooser.priority2Chooser.get(),
                        RobotContainer.autoChooser.priority3Chooser.get()
                    )));

                    // put("BLUE_AMP_4_CL3_SHIFT",  Pair.of(new CENTERLINE_NOTES[] { CENTERLINE_NOTES.H, CENTERLINE_NOTES.G, CENTERLINE_NOTES.F },
                    // () -> new BlueAmp4CenterLineOFSZ_SHIFT(
                    //     RobotContainer.autoChooser.priority1Chooser.get(), 
                    //     RobotContainer.autoChooser.priority2Chooser.get(),
                    //     RobotContainer.autoChooser.priority3Chooser.get()
                    // )));

                    // put("BLUE_AMP_4_CL3",  Pair.of(new CENTERLINE_NOTES[] { CENTERLINE_NOTES.H, CENTERLINE_NOTES.G, CENTERLINE_NOTES.F },
                    // () -> new BlueAmp4CenterLineOFSZ(
                    //     RobotContainer.autoChooser.priority1Chooser.get(), 
                    //     RobotContainer.autoChooser.priority2Chooser.get(),
                    //     RobotContainer.autoChooser.priority3Chooser.get()
                    // )));

                }});
                put(StartingLocation.CENTER, new HashMap<String, Pair<CENTERLINE_NOTES[], Supplier<AutoMode>>>() {{
            
                    put("ALT_BLUE_CENTER_CLEANUP_5", Pair.of(empty, AltBlueCleanup5NoteOFSZ::new));

                    put("BLUE_CENTER_CLEANUP_5", Pair.of(empty, BlueCleanup5OFSZ::new));

                    put("BLUE_CENTER_PJ-CL2", Pair.of(new CENTERLINE_NOTES[] { CENTERLINE_NOTES.E, CENTERLINE_NOTES.F, CENTERLINE_NOTES.G }, 
                    () -> new BlueCenter4OFSZ(
                        RobotContainer.autoChooser.priority1Chooser.get(), 
                        RobotContainer.autoChooser.priority2Chooser.get()
                    )));
                    
                }});
                put(StartingLocation.RIGHT, new HashMap<String, Pair<CENTERLINE_NOTES[], Supplier<AutoMode>>>() {{

                    put("BLUE_SOURCE_4_CL3_SHIFT", Pair.of(new CENTERLINE_NOTES[] { CENTERLINE_NOTES.D, CENTERLINE_NOTES.E, CENTERLINE_NOTES.F }, 
                    () -> new BlueSource4NoteOFSZ_SHIFT(
                        RobotContainer.autoChooser.priority1Chooser.get(), 
                        RobotContainer.autoChooser.priority2Chooser.get(),
                        RobotContainer.autoChooser.priority3Chooser.get()
                    )));

                    // put("BLUE_SOURCE_4_CL3", Pair.of(new CENTERLINE_NOTES[] { CENTERLINE_NOTES.D, CENTERLINE_NOTES.E, CENTERLINE_NOTES.F }, 
                    // () -> new BlueSource4NoteOFSZ(
                    //     RobotContainer.autoChooser.priority1Chooser.get(), 
                    //     RobotContainer.autoChooser.priority2Chooser.get(),
                    //     RobotContainer.autoChooser.priority3Chooser.get()
                    // )));

                }});
            }}
        );
    }

    public AutoChooser() {
        // autonInfo = Shuffleboard.getTab("DriverStation").add("Auton Info", "AUTON INFO").getEntry();

        this.colorSelector = new LoggedDashboardChooser<>("AutoChooser/Alliance Color");

        // --------- ALLIANCE COLOR AUTOMATICALLY SELECTS TO BLUE --------- //

        this.colorSelector.addDefaultOption("BLUE", AllianceColor.BLUE);
        this.colorSelector.addOption("RED", AllianceColor.RED);

        // --------- ALLIANCE COLOR AUTOMATICALLY SELECTS TO RED --------- //

        // this.colorSelector.addDefaultOption("RED", AllianceColor.RED);
        // this.colorSelector.addOption("BLUE", AllianceColor.BLUE);

        // --------------------------------------------------------------- //

        this.startingSideSelector = new LoggedDashboardChooser<>("AutoChooser/Starting Location");
        this.startingSideSelector.addDefaultOption("LEFT", StartingLocation.LEFT);
        this.startingSideSelector.addOption("CENTER", StartingLocation.CENTER);
        this.startingSideSelector.addOption("RIGHT", StartingLocation.RIGHT);

        this.routineChooser = new SwitchableChooser<String>("AutoChooser/Routine");
        
        this.commitAuton = new LoggedDashboardBoolean("AutoChooser/Commit Auton Config", false);
        this.selectedAutoNameDisplay = new LoggedDashboardString("AutoChooser/Selected Auto Name", "NO AUTO SELECTED");
        this.autoReadyDisplay = new LoggedDashboardBoolean("AutoChooser/Ready to run??", false);

        this.priority1Chooser = new SwitchableChooser<CENTERLINE_NOTES>("AutoChooser/Priority 1");
        this.priority2Chooser = new SwitchableChooser<CENTERLINE_NOTES>("AutoChooser/Priority 2");
        this.priority3Chooser = new SwitchableChooser<CENTERLINE_NOTES>("AutoChooser/Priority 3");

        this.frontLeftCameraStatus = new LoggedDashboardBoolean("AutoChooser/Front Camera Status", false);
        this.backLeftCameraStatus = new LoggedDashboardBoolean("AutoChooser/Back Camera Status", false);
        this.noteDetectorCameraStatus = new LoggedDashboardBoolean("AutoChooser/Note Detector Camera Status", false);
    }

    // /hud/AutoChooser/alliance, /start, /options
    // red or blue = alliance
    // left, center, right = start
    // stringified array []

    private void createAuto() {
        if (this.selectedAuto != null) {
            Drive.getInstance().setPose(this.selectedAuto.getStartingPose());

            Timer autoTimer = new Timer();
            this.selectedAutoCommand = this.selectedAuto.routine()
                .beforeStarting(RobotContainer.noteDetector::resetSimulatedPresence)
                .beforeStarting(this.selectedAuto::onStart)
                .beforeStarting(autoTimer::restart)
                .deadlineWith(Commands.run(this.selectedAuto::onLoop))
                .andThen(this.selectedAuto::onEnd)
                .andThen(() -> { 
                    System.out.println("Auto " + this.selectedAuto.getName() + " finished in " + autoTimer.get() + "s"); 
                    autoTimer.stop(); 
                });
            
            this.drawPathOnField();
            Visualizer.getInstance().setupAuto();
        }
    }

    public void startAuto() {
        if (this.selectedAutoCommand != null) {
            this.selectedAutoCommand.schedule();
        }
    }

    public void cleanup() {
        this.reset();

        if (this.selectedAuto != null) {
            this.selectedAutoCommand.cancel();
        }
    }

    public void periodic() {
        boolean setupChanged = startingSideSelector.get() != lastStartingSide || colorSelector.get() != lastAllianceColor;
        boolean routineChanged = routineChooser.get() == null || !routineChooser.get().equals(lastRoutine);
        boolean prioritiesChanged = priority1Chooser.get() != lastPriorityOptions[0] || priority2Chooser.get() != lastPriorityOptions[1] || priority3Chooser.get() != lastPriorityOptions[2];

        if (setupChanged || routineChanged || prioritiesChanged) {
            reset();
            setPriorityOptions();
        }

        if (prioritiesChanged) {
            cleanupPriorityOptions();
        }

        RobotStatus.getInstance().overrideAllianceColor(colorSelector.get());
        
        if (commitAuton.get()) {
            System.out.println("COMMIT BUTTON PRESSED");
            StartingLocation side = startingSideSelector.get();
            AllianceColor color = colorSelector.get();

            var choice = AUTO_MAP.get(color).get(side).get(routineChooser.get());
            if (choice != null) {
                AutoMode selected = choice.getSecond().get();
                this.selectedAuto = selected;
                this.createAuto();
                
                this.selectedAutoNameDisplay.set(routineChooser.get());
                this.autoReadyDisplay.set(true);

                // autonInfo.setString(this.selectedAuto.toString());
            }

            commitAuton.set(false);
        }

        lastStartingSide = startingSideSelector.get();
        lastAllianceColor = colorSelector.get();
        lastRoutine = routineChooser.get();
        lastPriorityOptions = new CENTERLINE_NOTES[] {priority1Chooser.get(), priority2Chooser.get(), priority3Chooser.get()};

        frontLeftCameraStatus.set(RobotContainer.photon.frontLeftCameraStatus());
        backLeftCameraStatus.set(RobotContainer.photon.backLeftCameraStatus());
        noteDetectorCameraStatus.set(RobotContainer.noteDetector.cameraStatus());
    }

    public void onReset(Runnable onReset) {
        this.onReset = onReset;
    }

    private void reset() {
        selectedAuto = null;
        autoReadyDisplay.set(false);
        selectedAutoNameDisplay.set("NO AUTO SELECTED");
        RobotContainer.drive.getField().getObject("Trajectory").setPoses(new Pose2d[] {});
        RobotContainer.drive.getField().getObject("NUMBER_1").setPoses(new Pose2d[] {});
        RobotContainer.drive.getField().getObject("NUMBER_2").setPoses(new Pose2d[] {});
        RobotContainer.drive.getField().getObject("NUMBER_3").setPoses(new Pose2d[] {});

        routineChooser.setOptions(
            AUTO_MAP.get(colorSelector.get()).get(startingSideSelector.get()).keySet().toArray(String[]::new)
        );
        
        if (this.onReset != null)
            this.onReset.run();
    }

    private void setPriorityOptions() {
        var options = AUTO_MAP.get(colorSelector.get()).get(startingSideSelector.get()).get(routineChooser.get()).getFirst();
        if (options.length != 0) {
            priority1Chooser.setOptions(options);
            // if (options.length > 1) {
            priority2Chooser.setOptions(options);
            // } else if(options.length == 3) {
            priority3Chooser.setOptions(options);
            // }
        } else {
            var empty = new CENTERLINE_NOTES[] { };
            priority1Chooser.setOptions(empty);
            priority2Chooser.setOptions(empty);
            priority3Chooser.setOptions(empty);
        }
    }

    private void cleanupPriorityOptions() {
        var options = AUTO_MAP.get(colorSelector.get()).get(startingSideSelector.get()).get(routineChooser.get()).getFirst();

        var firstChoice = priority1Chooser.get();
        var secondChoice = priority2Chooser.get();

        priority2Chooser.setOptions(Arrays.stream(options).filter(x -> x != firstChoice).toArray(CENTERLINE_NOTES[]::new));
        priority3Chooser.setOptions(Arrays.stream(options).filter(x -> x != firstChoice && x != secondChoice).toArray(CENTERLINE_NOTES[]::new));
    }

    private void drawPathOnField() {
        List<Pose2d> poses = new ArrayList<>();

        String pointsList = "[";

        int i = 0;
        int granularity = 25;
        for (PlannedPath path : this.selectedAuto.getRoute().getPaths()) {
            for (var state : path.getDriveTrajectory().getStates()) {
                poses.add((state.poseMeters));
                if (i % granularity == 0) {
                    if (i != 0) {
                        pointsList += ", ";
                    }
                    pointsList += "[" + ((int) (state.poseMeters.getX() / 100))  + ", " + (int) (state.poseMeters.getY() / 100) + "]"; 
                }
                i++;
            }
        }

        pointsList += "]";

        i = 1;
        for (CENTERLINE_NOTES notes : this.selectedAuto.priorityList()) {
            DrawOnField.number(
                notes.position.get()
                    .plus(new Translation2d(RobotStatus.getInstance().getAllianceColor() == AllianceColor.RED ? -1 : 1, 0)),
                i
            );
            i++;
        }

        RobotContainer.drive.getField().getObject("Trajectory").setPoses(poses.toArray(Pose2d[]::new));
        Logger.recordOutput("Swerve/SelectedAut∂o", pointsList);
    }

    public enum StartingLocation {
        LEFT,
        CENTER,
        RIGHT;
    }

    public enum CENTERLINE_NOTES {
        D(() -> FieldConfig.getInstance().NOTE_D, false),
        E(() -> FieldConfig.getInstance().NOTE_E, true), 
        F(() -> FieldConfig.getInstance().NOTE_F, true),
        G(() -> FieldConfig.getInstance().NOTE_G, true),
        H(() -> FieldConfig.getInstance().NOTE_H, false),
        NONE(() -> new Translation2d(0, 0), false);

        public Supplier<Translation2d> position;
        public boolean transitThroughStage = false;
        CENTERLINE_NOTES(Supplier<Translation2d> position, boolean transitThroughStage) {
            this.position = position;
            this.transitThroughStage = transitThroughStage;
        }
    }
}