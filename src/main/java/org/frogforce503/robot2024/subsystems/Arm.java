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

public class Arm extends SubsystemBase {
    private CANMotor arm;
    private DigitalInput leftArmZeroSwitch, rightArmZeroSwitch;

    private final double convFactor = 180.0;

    private ArmFeedforward armFFController;
    
    public final double MIN_POS = 0;
    public final double MAX_POS = 115;

    static Arm instance;

    public static Arm getInstance() {
        if (instance == null) { instance = new Arm(); }
        return instance;
    }

    public enum Goal {
        OFF(() -> new MotorSetpoint()),
        ZERO(() -> new MotorSetpoint(MotorControlMode.PercentOutput, RobotContainer.arm.getPosition() >= 22.5 && RobotContainer.wrist.getPosition() <= 60 ? -1 : -0.2)),
        AIMING(() -> {
            return new MotorSetpoint(MotorControlMode.Position, ShotPlanner.getInstance().getArmSetpoint());
        }),
        HINT(() -> new MotorSetpoint(MotorControlMode.Position, RobotContainer.arm.currentHint)),
        AMP(() -> new MotorSetpoint(MotorControlMode.Position, ShotPlanner.Presets.AMP.getArmAngle())),
        CLIMB_STEP_1(() -> new MotorSetpoint(MotorControlMode.Position, 50), 1),
        CLIMB_STEP_3(() -> new MotorSetpoint(MotorControlMode.Position, 107), 1),
        CLIMB_STEP_7(() -> new MotorSetpoint(MotorControlMode.Position, 95), 1),
        HAIL_MARY(() -> new MotorSetpoint(MotorControlMode.Position, ShotPlanner.Presets.HAIL_MARY.getArmAngle()));

        public Supplier<MotorSetpoint> setpoint;
        public int slot;
        private Goal(Supplier<MotorSetpoint> setpoint, int slot) {
            this.setpoint = setpoint;
            this.slot = slot;
        }

        private Goal(Supplier<MotorSetpoint> setpoint) {
            this.setpoint = setpoint;
            this.slot = 0;
        }
    }

    public Goal currentGoal = Goal.ZERO;
    private double currentHint = 0;

    public double testSetpoint = 0.0;
    private double currentSetpoint = 0.0;

    private double FFOutput = 0.0;

    public Arm() {
        leftArmZeroSwitch = new DigitalInput(Robot.bot.leftArmZeroSwitchID);
        rightArmZeroSwitch = new DigitalInput(Robot.bot.rightArmZeroSwitchID);

        arm = CANMotor.createSparkMax(Robot.bot.armID, MotorType.kBrushless, true);
        arm.configureSim(3.25, 360);

        arm.getSparkMax().clearFaults();
        arm.getSparkMax().restoreFactoryDefaults();

        arm.getSparkMax().setIdleMode(IdleMode.kBrake);
        arm.getSparkMax().setInverted(Robot.bot.armInverted);
        arm.getSparkMax().setPositionConversionFactor(convFactor);

        // Regular arm PID
        arm.setPIDF(0, Robot.bot.armP, Robot.bot.armI, Robot.bot.armD, Robot.bot.armFF);

        // High-movement arm PID
        arm.setPIDF(1, Robot.bot.armP_SLOT1, Robot.bot.armI_SLOT1, Robot.bot.armD_SLOT1, Robot.bot.armFF_SLOT1);

        arm.getSparkMax().selectProfileSlot(0);

        arm.getSparkMax().burnFlash();

        armFFController = new ArmFeedforward(0.0, 0.4, 0.0, 0.0);

        SBUtil.initSubsystemData("Arm", "Arm", arm, MotorControlMode.Position);

        SBUtil.initComponent("Arm", "Left Switch Pressed", this::leftSwitchPressed);
        SBUtil.initComponent("Arm", "Right Switch Pressed", this::rightSwitchPressed);

        SBUtil.initComponent("Arm", "Arm Test Setpoint", 0);
        SBUtil.initComponent("Arm", "Arm At Goal", this::atGoal);
        SBUtil.initComponent("Arm", "Goal", () -> currentGoal.name());
        SBUtil.initComponent("Arm", "Setpoint", () -> currentSetpoint);
        SBUtil.initComponent("Arm", "Commanded Signal", arm.getSparkMax()::getAppliedOutput);

        SBUtil.initComponent("COAST MODE", "ARM AND WRIST", false, 0, 0);

        SBUtil.initComponent("Arm", "FF/output", () -> FFOutput);
        SBUtil.initComponent("Arm", "FF/kS", Double.valueOf(0), 0, 0);
        SBUtil.initComponent("Arm", "FF/kG", Double.valueOf(0), 0, 0);
        SBUtil.initComponent("Arm", "FF/kV", Double.valueOf(0), 0, 0);
        SBUtil.initComponent("Arm", "FF/kA", Double.valueOf(0), 0, 0);
    }
    
    @Override
    public void periodic() {
        MotorSetpoint commandedSetpoint = currentGoal.setpoint.get();

        boolean goingDown = (commandedSetpoint.controlMode == MotorControlMode.Position ? getPosition() > commandedSetpoint.output : commandedSetpoint.output < 0);

        if (goingDown && isArmZeroed()) {
            currentGoal = Goal.OFF;
            commandedSetpoint = new MotorSetpoint();
        }
        
        if (commandedSetpoint.controlMode == MotorControlMode.Position) {
            commandedSetpoint.output = MathUtil.clamp(MathUtils.round(commandedSetpoint.output, 3), MIN_POS, MAX_POS);
        }

        if (isArmZeroed()) {
            if (getPosition() != 0)
                arm.setEncoderPosition(0);

            if (arm.getPercent() < 0) {
                commandedSetpoint = new MotorSetpoint();
            }
        }

        double error = Math.abs(commandedSetpoint.output - getPosition());
        SmartDashboard.putNumber("ERROR ON ARM", error);

        if (RobotContainer.climbStep >= 1) {
            arm.getSparkMax().selectProfileSlot(1);
        } else {
            arm.getSparkMax().selectProfileSlot(0);
        }

        if (arm.getPercent() >= 0 && arm.getPosition() >= MAX_POS) {
            arm.set(MotorControlMode.Position, MAX_POS - 2);
        }

        FFOutput = currentSetpoint == 0.0 ? 0.0 : armFFController.calculate(Units.degreesToRadians(commandedSetpoint.output - 14), 0.0, 0.0);
        FFOutput = RobotContainer.climbStep >= 1 ? 0.0 : FFOutput;
        arm.set(commandedSetpoint.controlMode, commandedSetpoint.output, FFOutput);
        currentSetpoint = commandedSetpoint.output;

        arm.update();

        testSetpoint = SBUtil.getComponent("Arm", "Arm Test Setpoint", 0);
        SBUtil.updateSubsytemData("Arm", "Arm", arm, MotorControlMode.Position);

        arm.getSparkMax().setIdleMode(
            SBUtil.getComponent("COAST MODE", "ARM AND WRIST", false) ? IdleMode.kCoast : IdleMode.kBrake
        );
        
        if (SBUtil.getComponent("Arm", "Arm Motor/Tuning PIDs?", true)) {
            double newKs = SBUtil.getComponent("Arm", "FF/kS", 0.0);
            double newKg = SBUtil.getComponent("Arm", "FF/kG", 0.0);
            double newKv = SBUtil.getComponent("Arm", "FF/kV", 0.0);
            double newKa = SBUtil.getComponent("Arm", "FF/kA", 0.0);

            if (armFFController.ka != newKa || armFFController.kg != newKg || armFFController.kv != newKv || armFFController.ks != newKs) {
                System.out.println("CHANGING FEEDFORWARDS");
                armFFController = new ArmFeedforward(newKs, newKg, newKv);
            }
        }
    }

    private boolean leftSwitchPressed() {
        if (RobotBase.isSimulation()) {
            return arm.getPosition() <= 0;
        }

        return !leftArmZeroSwitch.get();
    }

    private boolean rightSwitchPressed() {
        if (RobotBase.isSimulation()) {
            return arm.getPosition() <= 0;
        }
        
        return !rightArmZeroSwitch.get();
    }

    public boolean isArmZeroed() { 
        return leftSwitchPressed() || rightSwitchPressed(); 
    }

    public double getPosition() { 
        return arm.getPosition(); 
    }

    public boolean atGoal() {
        return currentGoal != Goal.ZERO ? Math.abs(arm.getPosition() - currentGoal.setpoint.get().output) < 0.25 : isArmZeroed();
    }

    public void down() {
        currentGoal = Goal.ZERO;
    }

    public Command aim() {
        return Commands.run(() -> currentGoal = Goal.AIMING, this).withName("Arm Aiming");
    }

    public Command autonHint(double distanceForHint) {
        return Commands.run(() -> {
            currentHint = ShotPlanner.getInstance().getArmAngleForDistance(distanceForHint);
            currentGoal = Goal.HINT;
        }, this).withName("Arm Hint");
    }

    public Command amp() {
        return Commands.runEnd(() -> currentGoal = Goal.AMP, () -> {
            if (RobotContainer.isAutoAim.getAsBoolean())
                currentGoal = Goal.AIMING;
            else
                currentGoal = Goal.ZERO;
        }, this).withName("Arm AMP");
    }

    public Command climbStep1() {
        return Commands.runOnce(() -> currentGoal = Goal.CLIMB_STEP_1, this).withName("Arm Climb Step 1");
    }

    public Command climbStep3() {
        return Commands.runOnce(() -> currentGoal = Goal.CLIMB_STEP_3, this).withName("Arm Climb Step 3");
    }

    public Command climbStep7() {
        return Commands.runOnce(() -> currentGoal = Goal.CLIMB_STEP_7, this).withName("Arm Climb Step 7");
    }

    public Command hailMary() {
        return Commands.runEnd(() -> currentGoal = Goal.HAIL_MARY, () -> {
            if (RobotContainer.isAutoAim.getAsBoolean())
                currentGoal = Goal.AIMING;
            else
                currentGoal = Goal.ZERO;
            
            RobotContainer.isHailMary = false;
        }).withName("Hail Mary");
    }

    public Command setArmDown() {
        return Commands.runOnce(() -> currentGoal = Goal.ZERO, this);
    }
}