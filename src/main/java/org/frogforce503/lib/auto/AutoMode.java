package org.frogforce503.lib.auto;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.function.Function;
import java.util.function.Supplier;

import org.frogforce503.lib.auto.AutoChooser.CENTERLINE_NOTES;
import org.frogforce503.lib.auto.follower.SwerveFollowPathCommand;
import org.frogforce503.lib.trajectory.PlannedPath;
import org.frogforce503.lib.trajectory.Route;
import org.frogforce503.lib.trajectory.Route.Tree;
import org.frogforce503.lib.trajectory.Waypoint.Tag;
import org.frogforce503.lib.util.Logic;
import org.frogforce503.lib.util.Triple;
import org.frogforce503.robot2024.RobotContainer;
import org.frogforce503.robot2024.RobotStatus;
import org.frogforce503.robot2024.RobotStatus.AllianceColor;
import org.frogforce503.robot2024.auto.AutoUtil.Waypoints;
import org.frogforce503.robot2024.fields.FieldConfig;
import org.frogforce503.robot2024.subsystems.NoteDetector;
import org.frogforce503.robot2024.subsystems.NoteDetector.SIDE;
import org.frogforce503.robot2024.subsystems.drive.Drive;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

import static edu.wpi.first.wpilibj2.command.Commands.deadline;
import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

public abstract class AutoMode {
    private boolean shouldBalance = false;

    // Pose2d: Shooting Pose    Double: Arm Angle    Double: Wrist Angle
    protected Triple<Pose2d, Double, Double> redSourceFarShot = new Triple<>(
        Waypoints.facingGoal(FieldConfig.getInstance().RED_STAGE_RIGHT.plus(new Translation2d(0, 1.25))),
        5.0,
        0.0);
    /*
    protected Triple<Pose2d, Double, Double> redSourceNearShot = new Triple<>(
        Waypoints.facingGoal(),
        0.0,
        null);
    protected Triple<Pose2d, Double, Double> redAmpFarShot = new Triple<>(
        Waypoints.facingGoal(),
        0.0,
        null);
    protected Triple<Pose2d, Double, Double> redAmpNearShot = new Triple<>(
        Waypoints.facingGoal(),
        0.0,
        null);


    protected Triple<Pose2d, Double, Double> blueSourceFarShot = new Triple<>(
        Waypoints.facingGoal(),
        0.0,
        null);
    protected Triple<Pose2d, Double, Double> blueSourceNearShot = new Triple<>(
        Waypoints.facingGoal(),
        0.0,
        null);
    protected Triple<Pose2d, Double, Double> blueAmpFarShot = new Triple<>(
        Waypoints.facingGoal(),
        0.0,
        null);
    protected Triple<Pose2d, Double, Double> blueAmpNearShot = new Triple<>(
        Waypoints.facingGoal(),
        0.0,
        null);

    */
    
    public abstract Command routine();
    public abstract Route getRoute();
    // public abstract String getDescription();
    // public abstract PlannedPath[] getPaths();

    public void onStart() {}
    public void onLoop() {}
    public void onEnd() {}

    public CENTERLINE_NOTES[] priorityList() {
        return new CENTERLINE_NOTES[] {};
    }

    protected SwerveFollowPathCommand drive(PlannedPath path) {
        return new SwerveFollowPathCommand(path);
    }

    protected Command drive(PlannedPath path, double seek) {
        return new SwerveFollowPathCommand(path, seek);
    }

    protected Command driveAndAimInTheLast(PlannedPath path, double time) {
        Timer timer = new Timer();
        return drive(path, () -> timer.hasElapsed(path.getTotalTimeSeconds() - time)).beforeStarting(Commands.runOnce(timer::start));
    }

    protected Command driveAndIntake(PlannedPath path) {
        Command seq = drive(path).deadlineWith(RobotContainer.intake()).andThen(Commands.print("DONE DRIVING"));
        seq.addRequirements(RobotContainer.drive, RobotContainer.intake);
        return seq;
    }

    protected Command driveAndIntake(PlannedPath path, Supplier<Boolean> isAiming) {
        Command seq = drive(path, isAiming).deadlineWith(RobotContainer.intake()).andThen(Commands.print("DONE DRIVING"));
        seq.addRequirements(RobotContainer.drive, RobotContainer.intake);
        return seq;
    }

    protected Command driveAndIntakeParallel(PlannedPath path, Supplier<Boolean> isAiming) {
        Command seq = drive(path, isAiming).alongWith(RobotContainer.intake()).andThen(Commands.print("DONE DRIVING")).until(RobotContainer.feeder::noteInExitSensor);
        seq.addRequirements(RobotContainer.drive, RobotContainer.intake);
        return seq;
    }

    protected Command driveAndFetchParallel(PlannedPath path, double fetchTime, Supplier<Boolean> isAiming) {
        Command seq = drive(path, isAiming).fetchAtEndFor(fetchTime).alongWith(RobotContainer.intake()).andThen(Commands.print("DONE DRIVING")).until(RobotContainer.feeder::noteInExitSensor);
        seq.addRequirements(RobotContainer.drive, RobotContainer.intake);
        return seq;
    }


    protected Command scoreCycle(PlannedPath grabNote, PlannedPath comeBackAndScore) {
        return driveAndIntake(grabNote).andThen(driveAndIntake(comeBackAndScore, () -> true));
    }

    protected Command driveAndCheckIfNotePresent(PlannedPath path, NoteDetector.SIDE side, AutoCycleData data) {
        Timer timer = new Timer();
        return drive(path, data.interrupted ? data.endTimestamp : 0).deadlineWith(
            Commands.sequence(
                Commands.runOnce(timer::start),
                Commands.waitUntil(() -> {
                    if (RobotStatus.getInstance().getAllianceColor() == AllianceColor.BLUE)
                        return RobotContainer.drive.getPose().getX() >= FieldConfig.getInstance().BLUE_WING_LINE;

                    return RobotContainer.drive.getPose().getX() <= FieldConfig.getInstance().RED_WING_LINE;
                }),
                Commands.runOnce(() -> { 
                    boolean notePresent = side == SIDE.CENTER; // RobotContainer.noteDetector.isNotePresent(side))
                    data.noteIsPresent = notePresent;
                    data.endTimestamp = timer.get();
                    if (!notePresent) {
                        data.interrupted = true;
                        data.interruptedOnPickup = true;
                    }
                }) 
            )
        );
    }

    protected Command scoreCycle(PlannedPath grabNote, PlannedPath comeBackAndScore, NoteDetector.SIDE side, AutoCycleData data) {
        Timer scoreTimer = new Timer();
        return Commands.sequence(
            Commands.print("STARTING FROM" + data.endTimestamp),
            runOnce(scoreTimer::start),
            driveAndCheckIfNotePresent(grabNote, side, data),
            drive(comeBackAndScore)
                .deadlineWith(Commands.runOnce(() -> { 
                    boolean notePresent = true;
                    data.succesfulPickup = notePresent;
                    if (!notePresent) {
                        data.interrupted = true;
                        data.interruptedOnPickup = false;
                    }
                }) //RobotContainer.feeder.noteInEntrySensor())
                    .beforeStarting(Commands.waitSeconds(1.0)))
        ).deadlineWith(RobotContainer.intake()).andThen(RobotContainer.shootSequence()).onlyWhile(data::shouldntCancelCycle).andThen(Commands.print(data.toString()));
    }

    protected Command fetchWithTimeout(double timeout) {
        return RobotContainer.drive.chaseNote().withTimeout(timeout).deadlineWith(RobotContainer.intake());
    }

    protected Command driveAndFetchFixed(PlannedPath path, double time) {
        return Commands.deadline(
            drive(path).fetchAtEndFor(time),
            RobotContainer.intake()
        );
        // return Commands.sequence(
        //     Commands.race(
        //         driveAndIntake(path),
        //         Commands.sequence(
        //             Commands.waitSeconds(path.getTotalTimeSeconds() - time),
        //             Commands.waitUntil(RobotContainer.noteDetector::cameraStatus)
        //         )
        //     ),
        //     RobotContainer.fetch()
        //     .onlyIf(Logic.and(RobotContainer.noteDetector::noteInView, RobotContainer.noteDetector::cameraStatus))
        // );
    }

    protected Command driveAndFetch(PlannedPath path, double timeBeforeEndToSeek) {
        // if (NoteDetector.NOTE_DETECTION_DISABLED || RobotBase.isSimulation())
        //     return driveAndIntake(path);
        
        return new SwerveFollowPathCommand(path, true).deadlineWith(RobotContainer.intake())
            .withTimeout(path.getTotalTimeSeconds() - timeBeforeEndToSeek)
            .andThen(
                RobotContainer.drive.chaseNote()
                    .until(() -> {
                        boolean red = RobotStatus.getInstance().getAllianceColor() == AllianceColor.RED;
                        double midline = FieldConfig.getInstance().FIELD_DIMENSIONS.getX()/2;
                        boolean tooFar = (red && RobotContainer.drive.getPose().getX() <= midline - 0.25) || (!red && RobotContainer.drive.getPose().getX() >= midline + 0.25);
                        boolean check = RobotContainer.feeder.noteInEntrySensor();
                        System.out.println("FETCH COMMAND CHECKING " + check);
                        return check;
                    }).withTimeout(timeBeforeEndToSeek + 0.25)
            );
    }

    protected Command driveAndFetchIntakeSeparate(PlannedPath path, double timeBeforeEndToSeek) {
        if (NoteDetector.NOTE_DETECTION_DISABLED || RobotBase.isSimulation())
            return drive(path);
        
        Command seq = new SwerveFollowPathCommand(path, true)
            .withTimeout(path.getTotalTimeSeconds() - timeBeforeEndToSeek)
            .andThen(
                Commands.parallel(
                    RobotContainer.drive.chaseNote(),
                    RobotContainer.driverfeedback.fetch()
                ).until(() -> {
                        boolean red = RobotStatus.getInstance().getAllianceColor() == AllianceColor.RED;
                        double midline = FieldConfig.getInstance().FIELD_DIMENSIONS.getX()/2;
                        boolean tooFar = (red && RobotContainer.drive.getPose().getX() <= midline - 0.25) || (!red && RobotContainer.drive.getPose().getX() >= midline + 0.25);
                        return tooFar || RobotContainer.feeder.noteInEntrySensor();
                }).withTimeout(timeBeforeEndToSeek + 0.25)
            );

        seq.addRequirements(RobotContainer.drive);
        return seq;
    }

    protected SwerveFollowPathCommand drive(PlannedPath path, Supplier<Boolean> isAiming) {
        return new SwerveFollowPathCommand(path, isAiming);
    }

    protected Command drive(Supplier<PlannedPath> path) {
        return new SwerveFollowPathCommand(path);
    }

    protected Command scoreMaybeBail(PlannedPath path, PlannedPath getNextNormal, PlannedPath bailForNext, double bailTime, int[] storage) {
        return Commands.sequence(
            Commands.race(
                Commands.sequence(
                    driveAndIntake(path).deadlineWith(RobotContainer.startAim()),
                    RobotContainer.shootSequence()
                ),
                Commands.sequence(
                    Commands.waitSeconds(bailTime),
                    Commands.waitUntil(() -> {
                        if (storage[0] == 0) { // have not checked yet
                            boolean missedPickup = false; // !RobotContainer.feeder.noteInExitSensor();
                            storage[1] = missedPickup ? 1 : 0;
                            return missedPickup;
                        } else { // only let it check once
                            return false;
                        }
                    })
                )
            ),

            Commands.print("RESULT OF INTERRUPTION: " + storage[1]),

            Commands.select(
                Map.of(0, driveAndFetchFixed(getNextNormal, 0.9).withTimeout(getNextNormal.getTotalTimeSeconds() - 0.5), 1, driveAndFetch(bailForNext, 0.75)),
                () -> storage[1]
            ) 
        );
    }

    protected Command scoreMaybeShift(PlannedPath scoreFirstPart2, double scoreFirstPart2_TIMEOUT, Pair<Double, Double> scoreFirstHint, PlannedPath pickupSecond, Pair<Double, Double> pickupSecondHint, PlannedPath shiftPath) {
        return Commands.either(
            Commands.sequence(
                driveAndIntake(scoreFirstPart2).alongWith(RobotContainer.hintAuton(scoreFirstHint)).withTimeout(scoreFirstPart2_TIMEOUT),
                
                RobotContainer.autonAdjust(),
                RobotContainer.shootSequence().beforeStarting(RobotContainer.hintAuton(scoreFirstHint, 0.05)),
                
                RobotContainer.hintAng(pickupSecondHint),
                driveAndFetchFixed(pickupSecond, 0.75)
            ),
            Commands.sequence(
                RobotContainer.hintAng(pickupSecondHint),
                driveAndFetchFixed(shiftPath, 0.5)   
            ),
            RobotContainer.feeder::noteInEntrySensor
        );
    }

    /**
     * @deprecated
     */
    // protected Command decide(PlannedPath[] options, Tag tag, double before, Supplier<Integer> which) {
    //     Supplier<Integer> which_ = () -> {tag.choose(which.get()); return tag.getChoice();};
    //     Timer timer = new Timer();
    //     return runOnce(timer::start).andThen(runOnce(tag::resetChoice), drive(() -> { 
    //         boolean elapsed = timer.hasElapsed(tag.getStamp() - before);
            
    //         PlannedPath chosen = options[elapsed && !tag.passed() ? which_.get() : tag.getChoice()];
    //         tag.pass(elapsed);
            
    //         return chosen;
    //     })).andThen(timer::stop);
    // }

    protected Command decide(Tree tree, Supplier<SIDE> which, Tag tag) {
        Supplier<SIDE> which_ = tag == null ? which : () -> {tag.choose(which.get()); return tag.getChoice();};

        Map<Integer, Command> options = new HashMap<>();
        Arrays.asList(tree.branches).forEach((branch) -> options.put(Integer.valueOf(options.size()), drive(branch)));

        return drive(tree.trunk)
            .andThen(Commands.select(options, () -> which_.get().priorityIndex()));
    }

    protected Command decide(Tree tree, Supplier<SIDE> which) {
        return decide(tree, which, null);
    }

    protected Command decide(Tree tree, Function<CENTERLINE_NOTES, SIDE> perspective, Tag tag) {
        HashMap<Integer, Command> options = new HashMap<>();
        options.put(0, tree.fallbackPath != null ? driveAndFetchFixed(tree.fallbackPath, 1.0) : Commands.none());
        for (int i = 0; i < tree.branches.length; i++) {
            options.put(i + 1, driveAndFetchFixed(tree.branches[i], 1.0));
        }

        return drive(tree.trunk).andThen(Commands.select(options, () -> {
            for (int i = 0; i < tree.branches.length; i++) {
                System.out.println("CHECKING NOTE " + tree.branchTiedNotes[i]);
                boolean noteThere = RobotContainer.noteDetector.isNotePresent(tree.branchTiedNotes[i], perspective);
                System.out.println("NOTE AT " + perspective.apply(tree.branchTiedNotes[i]) + ": " + noteThere);
                if (noteThere) {
                    System.out.println(i + "NOTE");
                    tag.select(tree.branchTiedNotes[i]);
                    return i + 1;
                }
            }
            tag.select(CENTERLINE_NOTES.NONE);
            return 0;
        })).andThen(print("DONE DECIDING"));
        // return drive(tree.trunk)
        //     .andThen(
        //     // Commands.either(

        //     // )    
            
        //     Commands.deferredProxy(() -> {
        //         for (int i = 0; i < tree.branches.length; i++) {
        //             boolean noteThere = RobotContainer.noteDetector.isNotePresent(perspective.apply(tree.branchTiedNotes[0]));
        //             noteThere = RobotBase.isSimulation() ? true : noteThere;
        //             if (noteThere) {
        //                 System.out.println(i + "NOTE");
        //                 tag.select(tree.branchTiedNotes[i]);
        //                 return drive(tree.branches[i]);
        //             }
        //         }
        //         return Commands.none();
        //     })).deadlineWith(RobotContainer.intake()).andThen(Commands.print("DONE DRIVING"));

    }

    protected Command returningDecision(Tree grabTree, Tree returnTree, Function<CENTERLINE_NOTES, SIDE> perspective, Tag choice) {

        HashMap<Integer, Command> firstOptions = new HashMap<>();
        for (int i = 0; i < returnTree.branches.length; i++) {
            firstOptions.put(i, driveAndIntake(returnTree.branches[i]));
        }

        Command getBackToDecisionPointPath = Commands.either(Commands.none(), Commands.select(firstOptions, () -> {
            for (int i = 0; i < returnTree.branches.length; i++) {
                if (choice.getNote() == returnTree.branchTiedNotes[i]) {
                    System.out.println("COMING BACK TO DECISIONPOINT FROM " + returnTree.branchTiedNotes[i]);
                    return i;
                }
            }
            return 0;
        }), () -> choice.getNote() == CENTERLINE_NOTES.NONE);

        HashMap<Integer, Command> secondOptions = new HashMap<>();
        secondOptions.put(0, Commands.print("DRIVINGBACKTOSCORE").andThen(drive(returnTree.trunk, () -> true)).andThen(RobotContainer.shootSequence()));
        for (int i = 0; i < grabTree.branches.length; i++) {
            secondOptions.put(i + 1, driveAndIntake(grabTree.branches[i]).andThen(driveAndIntake(returnTree.branches[i]), drive(returnTree.trunk, () -> true), RobotContainer.shootSequence()));
        }

        Command getAnotherOrGoScore = Commands.select(secondOptions,
            () -> {
                boolean gotNote = RobotContainer.feeder.noteInExitSensor();

                System.out.println("DO WE HAVE A NOTE:" + gotNote);

                if (gotNote) {
                    System.out.println("WE HAVE A NOTE, CONTINUING BACK");
                    return 0;
                }

                System.out.println("WE DON'T HAVE A NOTE, CHECKING FOR OTHER ONES");

                int currentChoice = 0;
                for (int i = 0; i < grabTree.branches.length; i++) {
                    if (choice.getNote() == grabTree.branchTiedNotes[i]) {
                        currentChoice = i;
                        System.out.println("WILL TRY REROUTING FOR NOTE " + grabTree.branchTiedNotes[i]);
                    }
                }

                for (int i = currentChoice + 1; i < grabTree.branches.length; i++) {
                    if (RobotContainer.noteDetector.isNotePresent(grabTree.branchTiedNotes[i], perspective)) {
                        return i + 1;
                    }
                }

                System.out.println("EXHAUSTED ALL OPTIONS");

                return 0;
            }
        ).beforeStarting(print("STARTING RETURN PATH")); //andThen(returningDecision(grabTree, returnTree, choice.select())); ideally we can run this recursivley, but this will take more time to figure out the logic for, until then just run the scoring path


        return Commands.sequence(
            print("GETTING BACK TO DECISIONPOINT"),
            getBackToDecisionPointPath,
            print("GO OUT FOR MORE OR COME BACK AND SCORE"),
            getAnotherOrGoScore
        );

        // return getBackToDecisionPointPath.andThen(Commands.select(secondOptions, () -> {
        //     for (int i = 0; i < grabTree.branches.length; i++) {
        //         if (choice.getNote() == grabTree.branchTiedNotes[i] && i < grabTree.branches.length - 1) {
        //             return i + 1;
        //         }
        //     }
        //     return 0;
        // }));

        // return Commands.deferredProxy(() -> {
        //     int currentBranch = 0;
        //     for (int i = 0; i < returnTree.branches.length; i++) {
        //         if (choice.getNote() == returnTree.branchTiedNotes[i]) {
        //             currentBranch = i;
        //         }
        //     }

        //     System.out.println("RETURNING TO " + currentBranch + " " + returnTree.branchTiedNotes[currentBranch]);

        //     boolean cantDoMore = currentBranch >= returnTree.branches.length - 1;

        //     return driveAndIntake(returnTree.branches[currentBranch]).andThen(
        //         Commands.either(
        //             drive(returnTree.trunk),
        //             driveAndIntake(grabTree.branches[currentBranch + 1]).andThen(returningDecision(grabTree, returnTree, choice.select(grabTree.branchTiedNotes[currentBranch+1]))),
        //             () -> false || cantDoMore //RobotContainer.feeder.noteInExitSensor()
        //         )
        //     );
        // }).beforeStarting(print("HIIIII"));
        // return drive(() -> {
        //     for (int i = 0; i < returnTree.branches.length; i++) {
        //         if (choice.getNote() == returnTree.branchTiedNotes[i]) {
        //             return returnTree.branches[i];
        //         }
        //     }
        //     return returnTree.branches[0];
        // }).deadlineWith(RobotContainer.intake());
    }

    protected Command decideAndIntake(Tree tree, Supplier<SIDE> which, Tag choice) {
        return decide(tree, which, choice).deadlineWith(RobotContainer.intake());
    }

    protected Command decideAndFetch(Tree tree, Supplier<SIDE> which, Tag tag) {
        Supplier<Integer> which_ = tag == null ? () -> which.get().priorityIndex() : () -> {tag.choose(which.get()); System.out.println("CHOOSING THE " + tag.getChoice()); return tag.getChoice().priorityIndex();};

        Map<Integer, Command> options = new HashMap<>();
        Arrays.asList(tree.branches).forEach((branch) -> options.put(Integer.valueOf(options.size()), driveAndFetch(branch, 0.85)));

        return drive(tree.trunk)
            .andThen(Commands.select(options, which_));        }

    protected Command decideAndIntake(Tree tree, Supplier<SIDE> which) {
        return decideAndIntake(tree, which, null);
    }

    protected Command choose(Tree tree, Supplier<SIDE> getChoice) {
        Map<Integer, Command> options = new HashMap<>();
        Arrays.asList(tree.branches).forEach((choice) -> options.put(Integer.valueOf(options.size()), drive(choice)));

        return Commands.runOnce(() -> System.out.println(getChoice.get())).andThen(Commands.select(options, () -> getChoice.get().priorityIndex()));
    }

    // protected Command decide(PlannedPath[] options, Tag tag, Supplier<SIDE> which) {
    //     return decide(options, tag, 0, which);
    // }

    /**
     * returns the current robot pose with a fallback for in simulation
     * every auto should start with a pose returned by this method
     * @param simulationFallback
     * @return either the current pose of the robot as detected by the AprilTags or, if the robot is simulated, the simulationFallback
     */
    protected Pose2d setupPose(Pose2d simulationFallback) {
        return RobotBase.isReal() ? RobotContainer.drive.getPose() : simulationFallback;
    }

    public String getName() {
        return this.getClass().getSimpleName();
    }

    public Pose2d getStartingPose() {
        return this.getRoute().getStartingPose();
    }

    public void setShouldBalance(boolean shouldBalance) {
        this.shouldBalance = shouldBalance;
    }

    protected boolean shouldBalance() {
        return this.shouldBalance;
    }

    public static class AutoCycleData {
        public boolean noteIsPresent = true;
        public boolean succesfulPickup = true;
        public double endTimestamp = 0.0;
        public boolean interrupted = false;
        public boolean interruptedOnPickup = false;

        public boolean shouldntCancelCycle() {
            return noteIsPresent && succesfulPickup;
        }

        public void reset() {
            noteIsPresent = true;
            succesfulPickup = true;
            endTimestamp = 0.0;
            interrupted = false;
            interruptedOnPickup = false;
        }

        public AutoCycleData() {
            reset();
        }

        @Override
        public String toString() {
            return "AutoCycleData [endTimestamp=" + endTimestamp + ", interrupted=" + interrupted + ", interruptedOnPickup="
                    + interruptedOnPickup + ", noteIsPresent=" + noteIsPresent + ", succesfulPickup=" + succesfulPickup + "]";
        }
    }
}