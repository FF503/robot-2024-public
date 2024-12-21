package org.frogforce503.robot2024.subsystems;

import java.util.function.Consumer;
import java.util.function.Supplier;

import org.frogforce503.lib.util.SBUtil;
import org.frogforce503.robot2024.Robot;
import org.frogforce503.robot2024.RobotContainer;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriverFeedback extends SubsystemBase {
    public CANdle ledLights;
    public CANdle frontLight;

    static DriverFeedback instance;

    public static DriverFeedback getInstance() {
        if (instance == null) { instance = new DriverFeedback(); }
        return instance;
    }

    public DriverFeedback() {
        ledLights = new CANdle(Robot.bot.candleID);
        frontLight = new CANdle(12);

        SBUtil.initComponent("DriverFeedback", "LEDColor", () -> currentGoal.currentColor.get().name());
    }

    public enum Goal {
        APRILTAG_DIAGNOSTICS(DriverFeedback::getColorBasedOnTags), 
        INTAKE(() -> LightColor.BLUE),
        OFF(() -> LightColor.OFF),
        FETCHING(() -> LightColor.PURPLE),
        CUSTOM(() -> RobotContainer.driverfeedback.customSelectedColor);

        public Supplier<LightColor> currentColor;
        Goal(Supplier<LightColor> currentColor) {
            this.currentColor = currentColor;
        }
    }
    
    public Goal currentGoal = Goal.APRILTAG_DIAGNOSTICS;
    private LightColor customSelectedColor = LightColor.OFF;

    @Override
    public void periodic() {
        currentGoal.currentColor.get().set.accept(ledLights);

        frontLight.setLEDs(255, 255, 255, 0, 0, 512);
    }

    private void setGoal(Goal goal) {
        ledLights.clearAnimation(0);
        currentGoal = goal;
    }

    public void setToDefault() {
        setGoal(Goal.APRILTAG_DIAGNOSTICS);
    }

    public Command signalIntakeGot() {
        return runEnd(() -> setGoal(Goal.INTAKE), this::setToDefault);
    }

    public Command off() {
        return runEnd(() -> setGoal(Goal.OFF), this::setToDefault);
    }

    public Command fetch() {
        return runEnd(() -> setGoal(Goal.FETCHING), this::setToDefault);
    }

    public void setToFetch() {
        setGoal(Goal.FETCHING);
    }

    public Command setColorManually(LightColor color) {
        return runEnd(() -> {
            customSelectedColor = color;
            setGoal(Goal.CUSTOM);
        }, this::setToDefault);
    }

    private static LightColor getColorBasedOnTags() {
        if (RobotContainer.photon.maxTagsFromOneCam() > 1) {
            return LightColor.GREEN;
        } else if (RobotContainer.photon.maxTagsFromOneCam() == 1) {
            return LightColor.YELLOW;
        } else {
            return LightColor.RED;
        }
    }

    public enum LightColor {
        RED(l -> l.setLEDs(255, 0, 0)), 
        GREEN(l -> l.setLEDs(0, 255, 0)), 
        GREEN_STROBE(l -> l.animate(new StrobeAnimation(0, 255, 0))), 
        GREEN_FLOW(l -> l.animate(new ColorFlowAnimation(0, 255, 0, 0, 0.6, 45, Direction.Forward))), 
        BLUE(l -> l.setLEDs(0, 0, 255)), 
        YELLOW(l -> l.setLEDs(244, 250, 12)), 
        PURPLE(l -> l.setLEDs(255, 0, 255)), 
        TEAL(l -> l.setLEDs(0, 255, 255)), 
        WHITE(l -> l.setLEDs(255, 255, 255)), 
        ORANGE(l -> l.setLEDs(250, 135, 5)), 
        RAINBOW(l -> l.animate(new RainbowAnimation())),
        FIRE(l -> l.animate(new FireAnimation())),
        OFF(l -> l.setLEDs(0, 0, 0));

        public Consumer<CANdle> set;
        LightColor(Consumer<CANdle> set) {
            this.set = set;
        }
    }
}