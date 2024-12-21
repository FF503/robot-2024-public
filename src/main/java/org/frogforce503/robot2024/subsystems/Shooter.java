package org.frogforce503.robot2024.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.frogforce503.lib.drivers.CANMotor;
import org.frogforce503.lib.drivers.CANMotor.MotorControlMode;
import org.frogforce503.lib.drivers.CANMotor.MotorSetpoint;
import org.frogforce503.lib.math.MathUtils;
import org.frogforce503.lib.util.SBUtil;
import org.frogforce503.robot2024.Robot;
import org.frogforce503.robot2024.RobotContainer;
import org.frogforce503.robot2024.fields.FieldConfig;
import org.frogforce503.robot2024.planners.ShotPlanner;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import static edu.wpi.first.units.MutableMeasure.mutable;

public class Shooter extends SubsystemBase {
    private CANMotor shooterMain;
    private CANMotor shooterFollower;
    
    private final boolean followerInverted = true;

    private static final double SHOOTING_SPEED = 4400;
    private static final double IDLE_SPEED = 1750;
    private static final double SPIT_SPEED = 1000;

    static Shooter instance;

    public static Shooter getInstance() {
        if (instance == null) { instance = new Shooter(); }
        return instance;
    }

    public enum GOAL {
        OFF(() -> new MotorSetpoint(MotorControlMode.PercentOutput, 0)),
        IDLE(() -> new MotorSetpoint(MotorControlMode.Velocity, IDLE_SPEED)),
        SHOOT(() -> new MotorSetpoint(MotorControlMode.Velocity, SHOOTING_SPEED)),
        AMP(() -> new MotorSetpoint(MotorControlMode.Velocity, 1500)),
        TRAP(() -> new MotorSetpoint(MotorControlMode.Velocity, 800)),
        SPIT(() -> new MotorSetpoint(MotorControlMode.Velocity, SPIT_SPEED)),
        REVERSE(() -> new MotorSetpoint(MotorControlMode.Velocity, -1250)),
        HAIL_MARY(() -> {
            double hailMary_kP = 365;

            Translation2d speaker = RobotContainer.red()
                ? FieldConfig.getInstance().getTagById(4).getTranslation()
                : FieldConfig.getInstance().getTagById(7).getTranslation();

            double distToHailMaryTarget = 
            RobotContainer.drive.getPose().getTranslation().minus(
                speaker.plus(new Translation2d(RobotContainer.red() ? Units.feetToMeters(-6) : Units.feetToMeters(6), 2)
            )).getNorm();

            double hailMarySpeed = MathUtils.clamp(hailMary_kP * distToHailMaryTarget, 1250, 3500); //3500

            Logger.recordOutput("Shooter/HailMarySpeed", hailMarySpeed);
            SmartDashboard.putNumber("Shooter HAILMARY Speed", hailMarySpeed);
            
            return new MotorSetpoint(MotorControlMode.Velocity, hailMarySpeed);
        });

        public Supplier<MotorSetpoint> setpoint;
        GOAL(Supplier<MotorSetpoint> setpoint) {
            this.setpoint = setpoint;
        }
    }

    public GOAL currentGoal = GOAL.IDLE;
    
    public Shooter() {
        shooterMain = CANMotor.createSparkMax(Robot.bot.topShooterWheelID, MotorType.kBrushless);
        shooterMain.configureSim(0.0, 0.0);

        shooterFollower = CANMotor.createSparkMax(Robot.bot.bottomShooterWheelID, MotorType.kBrushless);
        shooterFollower.configureSim(0.0, 0.0);

        shooterMain.getSparkMax().setIdleMode(IdleMode.kCoast);
        shooterMain.getSparkMax().setInverted(Robot.bot.shooterInverted);

        shooterFollower.getSparkMax().setIdleMode(IdleMode.kCoast);

        shooterMain.setPIDF(0, Robot.bot.shooterP, Robot.bot.shooterI, Robot.bot.shooterD, Robot.bot.shooterKv);
        shooterMain.getSparkMax().selectProfileSlot(0);

        shooterMain.setEncoderPosition(0);
        shooterFollower.setEncoderPosition(0);

        shooterFollower.getSparkMax().follow(shooterMain.getSparkMax(), followerInverted);
        
        shooterFollower.getSparkMax().burnFlash();
        shooterMain.getSparkMax().burnFlash();

        SBUtil.initSubsystemData("Shooter", "Shooter", shooterMain, MotorControlMode.Velocity);
        SBUtil.initSubsystemData("Shooter", "Shooter Follower", shooterFollower, MotorControlMode.Velocity);

        SBUtil.initComponent("Shooter", "At Setpoint", this::atTargetVelocity);
        SBUtil.initComponent("Shooter", "Shooter Test Setpoint", SHOOTING_SPEED);
        SBUtil.initComponent("Shooter", "Goal", () -> currentGoal.name());
        SBUtil.initComponent("Shooter", "PID Setpoint", () -> shooterMain.getSparkMax().getClosedLoopError() + shooterMain.getSparkMax().getEncoderVelocity());
        SBUtil.initComponent("Shooter", "At Hail Mary", this::atTargetHailMary);
    }

    @Override
    public void periodic() {
        SBUtil.updateSubsytemData("Shooter", "Shooter", shooterMain, MotorControlMode.Velocity);
        SBUtil.updateSubsytemData("Shooter", "Shooter Follower", shooterFollower, MotorControlMode.Velocity);

        MotorSetpoint setpoint = currentGoal.setpoint.get();
        shooterMain.set(setpoint.controlMode, setpoint.output);
    }

    public boolean atTargetVelocity() { 
        double cutoff = ShotPlanner.getInstance().getDistanceTarget() < 3 ? 2500 : SHOOTING_SPEED - 75;
        return RobotBase.isSimulation() || shooterMain.getVelocity() >= cutoff; 
    }

    public double getVelocity() {
        return shooterMain.getVelocity();
    }

    public boolean atTargetHailMary() {
        return RobotBase.isSimulation() || Math.abs(shooterMain.getVelocity() - GOAL.HAIL_MARY.setpoint.get().output) < 100;
    }

    public void idle() {
        currentGoal = GOAL.IDLE;
    }

    public void off() {
        currentGoal = GOAL.OFF;
    }

    public Command rampUp() {
        return Commands.runOnce(() -> currentGoal = GOAL.SHOOT, this);
    }

    public Command shootIntoAmp() {
        return Commands.runEnd(() -> currentGoal = GOAL.AMP, () -> {
            if (RobotContainer.isAutoAim.getAsBoolean())
                currentGoal = GOAL.SHOOT;
            else
                currentGoal = GOAL.IDLE;
        }, this);
    }

    public Command climbOff() {
        return Commands.runOnce(() -> currentGoal = GOAL.OFF, this);
    }

    public Command shootIntoTrap() {
        return Commands.run(() -> currentGoal = GOAL.TRAP, this);
    }

    @Deprecated
    public Command spit() {
        return Commands.runOnce(() -> currentGoal = GOAL.SPIT, this);
    }

    @Deprecated
    public boolean atSpitSpeed() {
        return RobotBase.isSimulation() || Math.abs(getVelocity() - SPIT_SPEED) < 100;
    }

    public Command poop() {
        return Commands.runOnce(() -> currentGoal = GOAL.IDLE, this);
    }

    public Command reverse() {
        return Commands.runOnce(() -> currentGoal = GOAL.REVERSE, this);
    }

    public Command hailMary() {
        return Commands.runEnd(() -> currentGoal = GOAL.HAIL_MARY, () -> {
            if (RobotContainer.isAutoAim.getAsBoolean())
                currentGoal = GOAL.SHOOT;
            else
                currentGoal = GOAL.IDLE;
        }, this);
    }

    public Command setToIdle() {
        return Commands.runOnce(() -> currentGoal = GOAL.IDLE, this);
    }

    // ------------------------------------ SYSID ------------------------------------ //

    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Angle> m_rotations = mutable(Rotations.of(0));
    private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));

    private SysIdRoutine shooterSysID = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(1).per(Seconds.of(1)), null, null,  (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> { 
                // shooterMain.set(MotorControlMode.PercentOutput, volts.in(Volts) / RobotController.getBatteryVoltage());
            },
            log -> {
                log.motor("shooter")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            shooterMain.getSparkMax().getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(m_rotations.mut_replace(shooterMain.getPosition(), Rotations))
                    .angularVelocity(m_velocity.mut_replace(shooterMain.getVelocity() / 60, RotationsPerSecond));
            },
            this
        )
    );

    public Command sysIdForwardQuasi() {
        return shooterSysID.quasistatic(Direction.kForward);
    }

    public Command sysIdForwardDynamic() {
        return shooterSysID.dynamic(Direction.kForward);
    }

    public Command sysIdReverseQuasi() {
        return shooterSysID.quasistatic(Direction.kReverse);
    }

    public Command sysIdReverseDynamic() {
        return shooterSysID.dynamic(Direction.kReverse);
    }
}