package org.frogforce503.robot2024.subsystems;

import java.util.function.Supplier;

import org.frogforce503.lib.drivers.CANMotor;
import org.frogforce503.lib.drivers.CANMotor.MotorControlMode;
import org.frogforce503.lib.drivers.CANMotor.MotorSetpoint;
import org.frogforce503.lib.math.MathUtils;
import org.frogforce503.lib.util.SBUtil;
import org.frogforce503.robot2024.Robot;
import org.frogforce503.robot2024.RobotContainer;
import org.frogforce503.robot2024.planners.ShotPlanner;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
    private CANMotor wrist;
    private DigitalInput zeroSwitch;

    private final double convFactor = 360;

    private ArmFeedforward wristFFController;
    private ArmFeedforward wristFFController_SMALLANGLES;

    public final double MIN_POS = 0;
    public final double[][] MAX_POSITIONS = {{0, 35}, {3, 110}, {7.5, 130}, {30, 150}, {60, 200}}; // corresponds the ARM position to the wrist maximum
    private double currentMax = MAX_POSITIONS[0][1];

    public Goal currentGoal = Goal.ZERO;

    private double currentWristHint = 0;
    private double currentSetpoint = 0;

    static Wrist instance;

    private double outputFF = 0.0;

    public static Wrist getInstance() {
        if (instance == null) { instance = new Wrist(); }
        return instance;
    }

    public enum Goal {
        OFF(() -> new MotorSetpoint()),
        ZERO(() -> new MotorSetpoint(MotorControlMode.PercentOutput, RobotContainer.wrist.getPosition() <= 7 ? -0.1 : -0.3)), 
        AIMING(() -> {
            return new MotorSetpoint(MotorControlMode.Position, ShotPlanner.getInstance().getWristSetpoint());
        }),
        AUTON_DOWN(() -> new MotorSetpoint(MotorControlMode.PercentOutput, RobotContainer.wrist.getPosition() <= 1 ? 0.0 : -0.05)),
        AUTON_HINT(() -> new MotorSetpoint(MotorControlMode.Position, RobotContainer.wrist.currentWristHint)),
        AMP(() -> new MotorSetpoint(MotorControlMode.Position, ShotPlanner.Presets.AMP.getWristAngle())),
        INTAKE_ASSIST(() -> new MotorSetpoint(MotorControlMode.Position, 10)),
        CLIMB_STEP_2(() -> new MotorSetpoint(MotorControlMode.Position, 150), 1),
        CLIMB_STEP_3(() -> new MotorSetpoint(MotorControlMode.Position, 150), 1),
        CLIMB_STEP_7(() -> new MotorSetpoint(MotorControlMode.Position, 135), 1),
        HAIL_MARY(() -> new MotorSetpoint(MotorControlMode.Position, ShotPlanner.Presets.HAIL_MARY.getWristAngle()));

        public Supplier<MotorSetpoint> setpoint;
        public int slot;
        private Goal(Supplier<MotorSetpoint> setpoint) {
            this.setpoint = setpoint;
            this.slot = 0;
        }

        private Goal(Supplier<MotorSetpoint> setpoint, int slot) {
            this.setpoint = setpoint;
            this.slot = slot;
        }
    }
    
    static int i = 0;
    public Wrist() {
        zeroSwitch = new DigitalInput(Robot.bot.leftWristZeroSwitchID);

        wrist = CANMotor.createSparkMax(Robot.bot.wristID, MotorType.kBrushless, true);
        wrist.configureSim(3.25, 360);

        wrist.getSparkMax().setIdleMode(IdleMode.kBrake);
        wrist.getSparkMax().setInverted(Robot.bot.wristInverted);
        wrist.getSparkMax().setPositionConversionFactor(convFactor);

        wrist.setPIDF(0, Robot.bot.wristP, Robot.bot.wristI, Robot.bot.wristD, Robot.bot.wristFF);
        wrist.setPIDF(1, Robot.bot.wristP_SLOT1, Robot.bot.wristI_SLOT1, Robot.bot.wristD_SLOT1, Robot.bot.wristFF_SLOT1);
        wrist.setPIDF(2, 0.007, 0.0, 0.0001, 0.0000); // front batter & cobra PID

        wrist.getSparkMax().selectProfileSlot(0);

        wrist.getSparkMax().setSmartCurrentLimit(35);

        wristFFController = new ArmFeedforward(Robot.bot.wristS, Robot.bot.wristG, Robot.bot.wristV, Robot.bot.wristA);
        wristFFController_SMALLANGLES = new ArmFeedforward(Robot.bot.wristS_SLOT1, Robot.bot.wristG_SLOT1, Robot.bot.wristV_SLOT1, Robot.bot.wristA_SLOT1);

        SBUtil.initSubsystemData("Wrist", "Wrist", wrist, MotorControlMode.Position);
        SBUtil.initComponent("Wrist", "Wrist Max", () -> currentMax);
        SBUtil.initComponent("Wrist", "Wrist Switch Pressed", this::limitSwitchPressed);
        SBUtil.initComponent("Wrist", "Goal", () -> currentGoal.name());
        SBUtil.initComponent("Wrist", "At Setpoint", this::atGoal);
        SBUtil.initComponent("Wrist", "Setpoint", () -> currentSetpoint);

        SBUtil.initComponent("Wrist", "FF/output", () -> this.outputFF);
    }
    
    @Override
    public void periodic() {
        currentMax = calculateWristMax();

        MotorSetpoint commandedSetpoint = currentGoal.setpoint.get();

        boolean goingDown = (commandedSetpoint.controlMode == MotorControlMode.Position ? getPosition() > commandedSetpoint.output : commandedSetpoint.output < 0);

        if (goingDown && isWristZeroed()) {
            currentGoal = Goal.OFF;
            commandedSetpoint = new MotorSetpoint();
        }
        
        if (commandedSetpoint.controlMode == MotorControlMode.Position) {
            commandedSetpoint.output = MathUtil.clamp(MathUtils.round(commandedSetpoint.output, 3), MIN_POS, currentMax);
        }

        if (isWristZeroed()) {
            wrist.setEncoderPosition(0);

            if (wrist.getPercent() < 0) {
                commandedSetpoint = new MotorSetpoint(MotorControlMode.PercentOutput, 0);
            }
        }
        
        if (wrist.getPercent() > 0 && wrist.getPosition() >= currentMax) {
            wrist.set(MotorControlMode.Position, currentMax);
        }

        double errorWrist = Math.abs(commandedSetpoint.output - getPosition());
        SmartDashboard.putNumber("ERROR ON WRIST", errorWrist);

        if (Math.abs(errorWrist) <= 5) {
            wrist.getSparkMax().selectProfileSlot(1);
        } else {
            wrist.getSparkMax().selectProfileSlot(0);
        }

        if (RobotContainer.operator.povRight().getAsBoolean() || RobotContainer.operator.povLeft().getAsBoolean()) {
            wrist.getSparkMax().selectProfileSlot(2);
        } else {
            wrist.getSparkMax().selectProfileSlot(0);
        }

        this.outputFF = currentSetpoint == 0.0 ? 0 : commandedSetpoint.controlMode != MotorControlMode.PercentOutput ? (Math.abs(errorWrist) >= 10 ? wristFFController : wristFFController_SMALLANGLES).calculate(Units.degreesToRadians(commandedSetpoint.output + 26 - RobotContainer.arm.getPosition()), 0.0, 0.0) : 0.0;
        this.outputFF = MathUtils.round(this.outputFF, 3);
        
        wrist.set(commandedSetpoint.controlMode, commandedSetpoint.output, this.outputFF);
        currentSetpoint = commandedSetpoint.output;
        
        SBUtil.updateSubsytemData("Wrist", "Wrist", wrist, MotorControlMode.Position);
        wrist.update();

        wrist.getSparkMax().setIdleMode(
            SBUtil.getComponent("COAST MODE", "ARM AND WRIST", false) ? IdleMode.kCoast : IdleMode.kBrake
        );
    }

    private double calculateWristMax() {
        double armPos = RobotContainer.arm.getPosition();
        for (int i = MAX_POSITIONS.length - 1; i >= 0; i--) {
            if (armPos >= MAX_POSITIONS[i][0]) {
                return MAX_POSITIONS[i][1];
            }
        }
        return MAX_POSITIONS[0][1];
    }

    public double getWristMax() {
        return currentMax;
    }

    private boolean limitSwitchPressed() {
        if (RobotBase.isSimulation()) {
            return wrist.getPosition() <= 0;
        }

        return !zeroSwitch.get();
    }

    public boolean isWristZeroed() { 
        return limitSwitchPressed(); 
    }

    public double getPosition() { 
        return wrist.getPosition(); 
    }

    public boolean atGoal() {
        return currentGoal != Goal.ZERO ? Math.abs(wrist.getPosition() - currentGoal.setpoint.get().output) < 0.75 : isWristZeroed(); // tolerance was 0.4
    }

    public void down() {
        currentGoal = Goal.ZERO;
    }

    public Command aim() {
        return Commands.run(() -> currentGoal = Goal.AIMING, this).withName("Wrist Aiming");
    }

    public Command autonHint(double distanceForHint) {
        return Commands.run(() -> {
            currentWristHint = ShotPlanner.getInstance().getWristAngleForDistance(distanceForHint);
            currentGoal = Goal.AUTON_HINT;
        }, this).withName("Wrist Auton Hint");
    }

    public Command amp() {
        return Commands.runEnd(() -> currentGoal = Goal.AMP, () -> {
            if (RobotContainer.isAutoAim.getAsBoolean())
                currentGoal = Goal.AIMING;
            else
                currentGoal = Goal.ZERO;
        }, this).withName("Wrist AMP");
    }

    @Deprecated
    public Command intakeAssist() {
        return Commands.runOnce(() -> ShotPlanner.getInstance().cancelHint()).andThen(Commands.runEnd(() -> currentGoal = Goal.INTAKE_ASSIST, this::down, this)).withName("Wrist Intake Assist");
    }

    public Command climbStep2() {
        return Commands.runOnce(() -> currentGoal = Goal.CLIMB_STEP_2, this).withName("Wrist Climb Step 2");
    }

    public Command climbStep3() {
        return Commands.runOnce(() -> currentGoal = Goal.CLIMB_STEP_3, this).withName("Wrist Climb Step 3");
    }

    public Command climbStep7() {
        return Commands.runOnce(() -> currentGoal = Goal.CLIMB_STEP_7, this).withName("Wrist Climb Step 7");
    }

    public Command hailMary() {
        return Commands.runEnd(() -> currentGoal = Goal.HAIL_MARY, () -> {
            if (RobotContainer.isAutoAim.getAsBoolean())
                currentGoal = Goal.AIMING;
            else
                currentGoal = Goal.ZERO;
        }, this).withName("High Hail Mary");
    }

    public Command setWristDown() {
        return Commands.runOnce(() -> currentGoal = Goal.ZERO, this);
    }
}