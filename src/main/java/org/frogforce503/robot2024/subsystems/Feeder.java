package org.frogforce503.robot2024.subsystems;

import java.util.function.Supplier;

import org.frogforce503.lib.drivers.CANMotor;
import org.frogforce503.lib.drivers.CANMotor.MotorControlMode;
import org.frogforce503.lib.drivers.CANMotor.MotorSetpoint;
import org.frogforce503.lib.util.Logic;
import org.frogforce503.lib.util.SBUtil;
import org.frogforce503.robot2024.Robot;
import org.frogforce503.robot2024.RobotContainer;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
    private CANMotor feeder;

    private final double feederConvFactor = 1;
    private final boolean feederInvert = true;

    private DigitalInput entryBeamBreak;
    private DigitalInput exitBeamBreak;

    private boolean wasInExitSensor = false;
    private double exitSensorEncoderPosition = 0;

    private LoggedDashboardBoolean simulationHasNote = new LoggedDashboardBoolean("Simulation Has Note");

    public enum Goal {
        OFF(() -> new MotorSetpoint()),
        INTAKE(() -> new MotorSetpoint(MotorControlMode.PercentOutput,  RobotContainer.feeder.noteInEntrySensor() ? 0.3 : (DriverStation.isAutonomousEnabled() ? 0.8 : 0.8))), // was 0.4 and .35 
        EJECT(() -> new MotorSetpoint(MotorControlMode.PercentOutput, -0.8)),
        SHOOT(() -> new MotorSetpoint(MotorControlMode.PercentOutput, 1.0)),
        TRAP_SHOOT(() -> new MotorSetpoint(MotorControlMode.PercentOutput, 0.8)),
        TRAP_PREPARE(() -> new MotorSetpoint(MotorControlMode.PercentOutput, RobotContainer.feeder.noteInExitSensor() ? -0.25 : 0.0));

        public Supplier<MotorSetpoint> setpoint;
        Goal(Supplier<MotorSetpoint> setpoint) {
            this.setpoint = setpoint;
        }
    }

    public Goal goal = Goal.OFF;

    public Feeder() {
        entryBeamBreak = new DigitalInput(Robot.bot.entryBeamBreakID);
        exitBeamBreak = new DigitalInput(Robot.bot.exitBeamBreakID);

        feeder = CANMotor.createSparkMax(Robot.bot.feederID, MotorType.kBrushless);
        feeder.configureSim(0.0, 0.0);

        feeder.getSparkMax().setIdleMode(IdleMode.kBrake);
        feeder.getSparkMax().setInverted(feederInvert);
        feeder.getSparkMax().setVelocityConversionFactor(feederConvFactor);
        
        feeder.setPIDF(0, Robot.bot.feederP, Robot.bot.feederI, Robot.bot.feederD, Robot.bot.feederFF);
        feeder.getSparkMax().selectProfileSlot(0);

        feeder.setEncoderPosition(0);

        SBUtil.initSubsystemData("Feeder", "Feeder", feeder, MotorControlMode.Velocity);

        SBUtil.initComponent("Feeder", "Exit Beam Break", this::noteInExitSensor);
        SBUtil.initComponent("Feeder", "Entry Beam Break", this::noteInEntrySensor);
        SBUtil.initComponent("Feeder", "Exit Sensor Encoder Position", () -> exitSensorEncoderPosition);
        SBUtil.initComponent("Feeder", "Goal", () -> goal.name());
        SBUtil.initComponent("Feeder", "Setpoint", () -> goal.setpoint.get().output);
    }

    @Override
    public void periodic() {
        if (noteInExitSensor() && !wasInExitSensor) {
            exitSensorEncoderPosition = feeder.getPosition();
        }

        wasInExitSensor = noteInExitSensor();

        if (goal == Goal.INTAKE && noteInExitSensor()) {
            goal = Goal.OFF;
        }

        if (RobotBase.isSimulation() && goal == Goal.SHOOT)
            setSimulatedNotePresent(false);

        MotorSetpoint setpoint = goal.setpoint.get();
        feeder.set(setpoint.controlMode, setpoint.output);

        SBUtil.updateSubsytemData("Feeder", "Feeder", feeder, MotorControlMode.Velocity);
    }

    public void off() {
        this.goal = Goal.OFF;
    }

    public Command intake() {
        return Commands.runEnd(() -> this.goal = Goal.INTAKE, this::off).until(this::noteInExitSensor).withName("Feeder Intake");
    }

    public Command eject() {
        return Commands.runEnd(() -> this.goal = Goal.EJECT, this::off);
    }

    public Command shoot() {
        return Commands.runEnd(() -> this.goal = Goal.SHOOT, this::off)
            .until(() -> !noteInExitSensor() && !noteInEntrySensor()).withName("Feeder Shoot");
    }

    public Command shootContinous() {
        return Commands.run(() -> this.goal = Goal.SHOOT);
    }

    public Command trapPrepare() {
        return Commands.runEnd(() -> this.goal = Goal.TRAP_PREPARE, this::off, this)
            .until(Logic.not(this::noteInExitSensor));
    }

    public Command trapShoot() {
        return Commands.runEnd(() -> this.goal = Goal.TRAP_SHOOT, this::off, this);
    }

    public boolean noteInEntrySensor() { 
        if (RobotBase.isSimulation())
            return simulationHasNote.get();
        return (entryBeamBreak.get());
    }

    public boolean noteInExitSensor() { 
        if (RobotBase.isSimulation())
            return simulationHasNote.get();
        return (exitBeamBreak.get());
    }

    public void setSimulatedNotePresent(boolean present) {
        simulationHasNote.set(present);
    }
}