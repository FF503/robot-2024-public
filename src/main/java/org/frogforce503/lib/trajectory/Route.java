package org.frogforce503.lib.trajectory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import org.frogforce503.lib.auto.AutoChooser.CENTERLINE_NOTES;
import org.frogforce503.robot2024.subsystems.drive.Drive;

import edu.wpi.first.math.geometry.Pose2d;

public class Route {
    List<PlannedPath> paths;
    boolean knownStart = false;

    public Route() {
        this.paths = new ArrayList<>();
    }

    public Route(PlannedPath ...paths) {
        this.paths = new ArrayList<>();
        this.paths.addAll(Arrays.asList(paths));
    }

    public Route addPaths(PlannedPath ...pathsIn) {
        for (PlannedPath path : pathsIn) {
            paths.add(path);
        }
        return this;
    }

    public Route addPathOptions(HashMap<CENTERLINE_NOTES, PlannedPath> pathsIn) {
        for (PlannedPath path : pathsIn.values()) {
            paths.add(path);
        }
        return this;
    }

    public Route addTrees(Tree ...trees) {
        for (Tree tree : trees) {
            if (tree.trunk != null)
                this.paths.add(tree.trunk);
            
            this.paths.addAll(Arrays.asList(tree.branches));
        }
        return this;
    }

    public Route withKnownStart() {
        this.knownStart = true;
        return this;
    }

    public List<PlannedPath> getPaths() {
        return paths;
    }

    public boolean hasKnownStart() {
        return this.knownStart;
    }

    public Pose2d getStartingPose() {
        if (this.paths.size() == 0)
            return new Pose2d();

        var p = this.getPaths().get(0);
        return (p != null ? p.getInitialHolonomicPose() : Drive.getInstance().getPose());
    }

    public static class Tree {
        public PlannedPath trunk;
        public PlannedPath[] branches;
        public CENTERLINE_NOTES[] branchTiedNotes;
        public PlannedPath fallbackPath;

        public Tree withTrunk(PlannedPath trunk) {
            this.trunk = trunk;
            return this;
        }

        public Tree withBranches(PlannedPath... branches) {
            this.branches = branches;
            return this;
        }

        public Tree withNotes(CENTERLINE_NOTES... notes) {
            this.branchTiedNotes = notes;
            return this;
        }

        public Tree withFallbackPath(PlannedPath path) {
            this.fallbackPath = path;
            return this;
        }
    }
}
