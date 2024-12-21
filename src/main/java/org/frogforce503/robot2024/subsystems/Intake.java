package org.frogforce503.robot2024.subsystems;

import java.util.function.Supplier;

import org.frogforce503.lib.drivers.CANMotor;
import org.frogforce503.lib.drivers.CANMotor.MotorControlMode;
import org.frogforce503.lib.drivers.CANMotor.MotorSetpoint;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import org.frogforce503.lib.util.SBUtil;
import org.frogforce503.robot2024.Robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final double convFactor = 1;
    private final boolean invert = true;

    private static final double INTAKE_SPEED = 5500;
    private static final double INTAKE_GEAR_RATIO = 24.0/36.0;

    private CANMotor intake;

    static Intake instance;

    public static Intake getInstance() {
        if (instance == null) { instance = new Intake(); }
        return instance;
    }

    public enum Goal {
        OFF(() -> new MotorSetpoint()),
        INTAKE(() -> new MotorSetpoint(MotorControlMode.Velocity, INTAKE_SPEED)),
        EJECT(() -> new MotorSetpoint(MotorControlMode.Velocity, -INTAKE_SPEED)),
        SHOOT_ASSIST(() -> new MotorSetpoint(MotorControlMode.Velocity, 5000));

        public Supplier<MotorSetpoint> setpoint;
        Goal(Supplier<MotorSetpoint> setpoint) {
            this.setpoint = setpoint;
        }
    }

    public Goal goal = Goal.OFF;

    public Intake() {
        intake = CANMotor.createSparkMax(Robot.bot.intakeID, MotorType.kBrushless);
        intake.configureSim(0.0, 0.0);

        intake.getSparkMax().setIdleMode(IdleMode.kCoast);
        intake.getSparkMax().setInverted(invert);
        intake.getSparkMax().setVelocityConversionFactor(convFactor);

        intake.setPIDF(0, Robot.bot.intakeP, Robot.bot.intakeI, Robot.bot.intakeD, Robot.bot.intakeFF);
        intake.getSparkMax().selectProfileSlot(0);

        intake.setEncoderPosition(0);
    
        SBUtil.initSubsystemData("Intake", "Intake", intake, MotorControlMode.Velocity);
        SBUtil.initComponent("Intake", "Goal", () -> goal.name());
        SBUtil.initComponent("Intake", "Velocity At Output", () -> intake.getVelocity() * INTAKE_GEAR_RATIO);
    }

    @Override
    public void periodic() {
        SBUtil.updateSubsytemData("Intake", "Intake", intake, MotorControlMode.Velocity);

        MotorSetpoint setpoint = goal.setpoint.get();
        intake.set(setpoint.controlMode, setpoint.output);
    }

    private void off() {
        this.goal = Goal.OFF;
    }

    public Command intake() {
        return runEnd(() -> this.goal = Goal.INTAKE, this::off);
    }

    public Command eject() {
        return runEnd(() -> this.goal = Goal.EJECT, this::off);
    }

    public Command shootAssist() {
        return runEnd(() -> this.goal = Goal.SHOOT_ASSIST, this::off);
    }
}