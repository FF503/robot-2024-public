package org.frogforce503.robot2024.auto.tuning;

import org.frogforce503.lib.auto.AutoMode;
import org.frogforce503.lib.swerve.ModifiedSignalLogger;
import org.frogforce503.lib.swerve.SwerveCommand;
import org.frogforce503.lib.swerve.SwerveCommand.CharacterizationCommand.Motor;
import org.frogforce503.lib.trajectory.PlannedPath;
import org.frogforce503.lib.trajectory.Route;
import org.frogforce503.lib.trajectory.SwervePathBuilder;
import org.frogforce503.lib.trajectory.Waypoint;
import org.frogforce503.robot2024.RobotContainer;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import static edu.wpi.first.units.Units.*;

public class CharacterizationAuto extends AutoMode {

    PlannedPath dummy = SwervePathBuilder.generate(3, 3, new Waypoint(), new Waypoint(new Translation2d(1, 0)));
    SysIdRoutine routine;

    SendableChooser<Command> commandChooser;

    private boolean usingFOC = true;

    private SysIdRoutine driveSysId = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(1.5).per(Seconds.of(2)), null, null, ModifiedSignalLogger.logState()),
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> { 
                RobotContainer.drive.acceptSwerveCommand(new SwerveCommand.CharacterizationCommand(Motor.DRIVE, usingFOC).withTargetVoltage(volts.in(Volts)));
            },
            null,
            RobotContainer.drive
        )
    );


    private SysIdRoutine driveSysIdReverse = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(1.5).per(Seconds.of(2)), null, null, ModifiedSignalLogger.logState()),
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> { 
                RobotContainer.drive.acceptSwerveCommand(new SwerveCommand.CharacterizationCommand(Motor.DRIVE, usingFOC).withTargetVoltage(volts.in(Volts)));
            },
            null,
            RobotContainer.drive
        )
    );

    private SysIdRoutine steerSysId = new SysIdRoutine(
        new SysIdRoutine.Config(null, null, null, ModifiedSignalLogger.logState()),
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> { 
                RobotContainer.drive.acceptSwerveCommand(new SwerveCommand.CharacterizationCommand(Motor.STEER, usingFOC).withTargetVoltage(volts.in(Volts)));
            },
            null,
            RobotContainer.drive
        )
    );

    private SysIdRoutine slipSysId = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(0.25).per(Seconds.of(1)), null, null, ModifiedSignalLogger.logState()),
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> { 
                RobotContainer.drive.acceptSwerveCommand(new SwerveCommand.CharacterizationCommand(Motor.DRIVE, usingFOC).withTargetVoltage(volts.in(Volts)));
            },
            null,
            RobotContainer.drive
        )
    );

    public CharacterizationAuto() {
        commandChooser = new SendableChooser<>();
        commandChooser.setDefaultOption("Drive Quasi Forward", driveSysId.quasistatic(Direction.kForward));
        commandChooser.addOption("Drive Quasi Reverse", driveSysIdReverse.quasistatic(Direction.kReverse));
        commandChooser.addOption("Drive Dynam Forward", driveSysId.dynamic(Direction.kForward));
        commandChooser.addOption("Drive Dynam Reverse", driveSysIdReverse.dynamic(Direction.kReverse));

        commandChooser.addOption("Steer Quasi Forward", steerSysId.quasistatic(Direction.kForward));
        commandChooser.addOption("Steer Quasi Reverse", steerSysId.quasistatic(Direction.kReverse));
        commandChooser.addOption("Steer Dynam Forward", steerSysId.dynamic(Direction.kForward));
        commandChooser.addOption("Steer Dynam Reverse", steerSysId.dynamic(Direction.kReverse));

        commandChooser.addOption("Drive Slip", slipSysId.quasistatic(Direction.kReverse));
        
        Shuffleboard.getTab("DriverStation").add(commandChooser);
    }

    @Override
    public Command routine() {
        return Commands.deferredProxy(commandChooser::getSelected);
    }

    @Override
    public Route getRoute() {
        return new Route(dummy);
    }
}