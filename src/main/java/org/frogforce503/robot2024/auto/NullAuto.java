package org.frogforce503.robot2024.auto;

import org.frogforce503.lib.auto.AutoMode;
import org.frogforce503.lib.trajectory.PlannedPath;
import org.frogforce503.lib.trajectory.Route;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class NullAuto extends AutoMode {

    @Override
    public Command routine() {
        return new InstantCommand(() -> System.out.println("ERROR: NULL AUTO"));
    }

    @Override
    public Route getRoute() {
        throw new UnsupportedOperationException("Unimplemented method 'getPath'");
    }

}