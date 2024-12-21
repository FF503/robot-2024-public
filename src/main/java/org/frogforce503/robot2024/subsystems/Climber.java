package org.frogforce503.robot2024.subsystems;

import org.frogforce503.lib.drivers.CANMotor;
import org.frogforce503.lib.drivers.CANMotor.MotorControlMode;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import org.frogforce503.lib.util.SBUtil;
import org.frogforce503.robot2024.Robot;
import org.frogforce503.robot2024.RobotContainer;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final double convFactor = 1/18;
    private final boolean leftInvert = false, rightInvert = true;

    private CANMotor climbLeftMotor, climbRightMotor;
    private DigitalInput leftZeroSwitch, rightZeroSwitch;

    private double EXTEND_POSITION = 94.0;

    private double BACKWARDCLIMB_POSITION = 42.0;

    private double ZERO_POWER = -0.5; // -0.2
    private double EXTEND_POWER = 0.5;
    private double EXTEND_POWER_BACKWARD = 0.75;
    private double RETRACT_POWER = -0.5;
    private double HOLD_POWER = -0.8;

    static Climber instance;

    public static Climber getInstance() {
        if (instance == null) { instance = new Climber(); }
        return instance;
    }

    public Climber() {
        leftZeroSwitch = new DigitalInput(Robot.bot.leftClimberZeroSwitchID);
        rightZeroSwitch = new DigitalInput(Robot.bot.rightClimberZeroSwitchID);

        climbLeftMotor = CANMotor.createSparkMax(Robot.bot.leftClimberID, MotorType.kBrushless, false);

        climbLeftMotor.getSparkMax().setIdleMode(IdleMode.kBrake);
        climbLeftMotor.getSparkMax().setInverted(leftInvert);
        climbLeftMotor.getSparkMax().setPositionConversionFactor(convFactor);

        climbLeftMotor.setPIDF(0, Robot.bot.climberP, Robot.bot.climberI, Robot.bot.climberD, Robot.bot.climberFF);
        climbLeftMotor.getSparkMax().selectProfileSlot(0);

        climbRightMotor = CANMotor.createSparkMax(Robot.bot.rightClimberID, MotorType.kBrushless);

        climbRightMotor.getSparkMax().setIdleMode(IdleMode.kBrake);
        climbRightMotor.getSparkMax().setInverted(rightInvert);
        climbRightMotor.getSparkMax().setPositionConversionFactor(convFactor);

        climbRightMotor.setPIDF(0, Robot.bot.climberP, Robot.bot.climberI, Robot.bot.climberD, Robot.bot.climberFF);
        climbRightMotor.getSparkMax().selectProfileSlot(0);

        climbLeftMotor.configureSim(0, 0); // Climbing doesn't work in sim
        climbRightMotor.configureSim(0, 0);

        climbLeftMotor.setEncoderPosition(0);
        climbRightMotor.setEncoderPosition(0);

        SBUtil.initSubsystemData("Climber Left", "Climber Left", climbLeftMotor, MotorControlMode.Position);
        SBUtil.initSubsystemData("Climber Right", "Climber Right", climbRightMotor, MotorControlMode.Position);
    
        SBUtil.initComponent("Climber Left", "Left Switch", () -> isLeftZeroed());
        SBUtil.initComponent("Climber Right", "Right Switch", () -> isRightZeroed());
        SBUtil.initComponent("Climber Right", "Step", () -> RobotContainer.climbStep);
        
        SBUtil.initComponent("COAST MODE", "CLIMBERS", false, 0, 0);
    }

    @Override
    public void periodic() {
        SBUtil.updateSubsytemData("Climber Left", "Climber Left", climbLeftMotor, MotorControlMode.Position);
        SBUtil.updateSubsytemData("Climber Right", "Climber Right", climbRightMotor, MotorControlMode.Position);

        climbLeftMotor.getSparkMax().setIdleMode(
            SBUtil.getComponent("COAST MODE", "CLIMBERS", false) ? IdleMode.kCoast : IdleMode.kBrake
        );

        climbRightMotor.getSparkMax().setIdleMode(
            SBUtil.getComponent("COAST MODE", "CLIMBERS", false) ? IdleMode.kCoast : IdleMode.kBrake
        );
    }

    public boolean isLeftZeroed() { 
        return !leftZeroSwitch.get(); 
    }

    public boolean isRightZeroed() { 
        return !rightZeroSwitch.get(); 
    }

    public double getLeftPos() { 
        return climbLeftMotor.getPosition(); 
    }
    public double getRightPos() { 
        return climbRightMotor.getPosition(); 
    }

    public void setLeftPct(double pct) {
        System.out.println("PULLING DOWN AT " + pct);
        climbLeftMotor.set(MotorControlMode.PercentOutput, pct);
    }
    
    public void setRightPct(double pct) {
        climbRightMotor.set(MotorControlMode.PercentOutput, pct);
    }

    /**
     * Zeroes left climber
     */
    public Command leftUnbucklePct() {
        return Commands.run(() -> setLeftPct(getLeftPos() < 15 ? ZERO_POWER / 5 : ZERO_POWER))
        .until(() -> isLeftZeroed())
        .andThen(Commands.runOnce(() -> {
            System.out.println("REZEROED LEFT");
            setLeftPct(0);
            climbLeftMotor.setEncoderPosition(0);
        }));
    }

    public Command leftExtendPct() {
        return Commands.run(() -> setLeftPct(EXTEND_POWER))
        .until(() -> getLeftPos() >= EXTEND_POSITION)
        .andThen(Commands.runOnce(() -> {
            climbLeftMotor.getSparkMax().setIdleMode(IdleMode.kBrake);
            setLeftPct(0);
        }));
    }

    public Command leftRetractPct() {
        return Commands.run(() -> setLeftPct(RETRACT_POWER))
        .until(() -> isLeftZeroed()).withTimeout(9)
        .andThen(Commands.runOnce(() -> {
            System.out.println("LEFT RETRACTING");
            climbLeftMotor.getSparkMax().setIdleMode(IdleMode.kBrake);
            setLeftPct(0);
            climbLeftMotor.setEncoderPosition(0);
        }))
        .andThen(Commands.run(() -> {
            setLeftPct(isLeftZeroed() ? 0 : HOLD_POWER);
        }));
    }

    public Command leftExtendPct_BACKWARD() {
        return Commands.run(() -> setLeftPct(EXTEND_POWER_BACKWARD))
        .until(() -> getLeftPos() >= EXTEND_POSITION)
        .andThen(Commands.runOnce(() -> {
            climbLeftMotor.getSparkMax().setIdleMode(IdleMode.kBrake);
            setLeftPct(0);
        }));
    }

    public Command leftRetractPct_BACKWARD() {
        return Commands.run(() -> setLeftPct(RETRACT_POWER))
        .until(() -> getLeftPos() <= BACKWARDCLIMB_POSITION).withTimeout(9)
        .andThen(Commands.runOnce(() -> {
            climbRightMotor.getSparkMax().setIdleMode(IdleMode.kBrake);
            setLeftPct(-0.2);
        }));
    }

    /**
     * Zeroes right climber
     */
    public Command rightUnbucklePct() {
        return Commands.run(() -> setRightPct(getRightPos() < 15 ? ZERO_POWER / 5 : ZERO_POWER))
        .until(() -> isRightZeroed())
        .andThen(Commands.runOnce(() -> {
            System.out.println("REZEROED LEFT");
            setRightPct(0);
            climbRightMotor.setEncoderPosition(0);
        }));
    }

    public Command rightExtendPct() {
        return Commands.run(() -> setRightPct(EXTEND_POWER))
        .until(() -> getRightPos() >= EXTEND_POSITION)
        .andThen(Commands.runOnce(() -> {
            climbRightMotor.getSparkMax().setIdleMode(IdleMode.kBrake);
            setRightPct(0);
        }));
    }

    public Command rightRetractPct() {
        return Commands.run(() -> setRightPct(RETRACT_POWER))
        .until(() -> isRightZeroed()).withTimeout(9)
        .andThen(Commands.runOnce(() -> {
            System.out.println("RIGHT RETRACTING");
            climbRightMotor.getSparkMax().setIdleMode(IdleMode.kBrake);
            setRightPct(0);
            climbRightMotor.setEncoderPosition(0);
        }))
        .andThen(Commands.run(() -> {
            setRightPct(isRightZeroed() ? 0 : HOLD_POWER);
        }));
    }

    public Command rightExtendPct_BACKWARD() {
        return Commands.run(() -> setRightPct(EXTEND_POWER_BACKWARD))
        .until(() -> getRightPos() >= EXTEND_POSITION)
        .andThen(Commands.runOnce(() -> {
            climbRightMotor.getSparkMax().setIdleMode(IdleMode.kBrake);
            setRightPct(0);
        }));
    }

    public Command rightRetractPct_BACKWARD() {
        return Commands.run(() -> setRightPct(RETRACT_POWER))
        .until(() -> getRightPos() <= BACKWARDCLIMB_POSITION).withTimeout(9)
        .andThen(Commands.runOnce(() -> {
            climbRightMotor.getSparkMax().setIdleMode(IdleMode.kBrake);
            setRightPct(-0.2);
        }));
    }

}