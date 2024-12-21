package org.frogforce503.robot2024;

import org.frogforce503.robot2024.subsystems.drive.Drive;

import java.sql.Driver;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

@SuppressWarnings("unused")
public class OI {
    static GenericHID driver = new GenericHID(0);
    static GenericHID buttonBoard = new GenericHID(1);
    static GenericHID overrideController = new GenericHID(2);


    // static DoubleSolenoid intakeShifter = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 0);
    // static DoubleSolenoid claw3 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 3, 5);
    // static DoubleSolenoid claw4 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 5, 2);
    // static DoubleSolenoid wrist = new DoubleSolenoid(PneumaticsModuleType.REVPH, 7, 6);

    static final int buttonA = 1;
    static final int buttonB = 2;
    static final int buttonX = 3;
    static final int buttonY = 4;
    static final int buttonLB = 5;
    static final int buttonRB = 6;
    static final int buttonSelect = 7;
    static final int buttonMenu = 8;
    static final int buttonLeftJoystick = 9;
    static final int buttonRightJoystick = 10;
    
    //button box
    static final int bbGridLeft = 11;
    static final int bbGridCenter = 10;
    static final int bbGridRight = 5;
    static final int bbCellLeft = 6;
    static final int bbCellCenter = 9;
    static final int bbCellRight = 4;
    static final int bbHigh = 3;
    static final int bbMid = 2;
    static final int bbLow = 1;
    static final int bbSignalCone = 8;
    static final int bbSignalCube = 12;
    static final int bbScoreButton = 7;

    static int grid = 3;
    static int cell = 0;
    static boolean high = false;

    static int targetPos = 0;
    
    static boolean driverRTWasPressed = false;
    static boolean overrideRTWasPressed = false;
    static boolean driverLTWasPressed = false;

    public static void buttonCheck(){
        // zero gyro
        // whenPressed(driver, buttonB, () -> Drive.getInstance()); // put back

    }

    public static void whileHeld(GenericHID controller, int button, Runnable whileHeld){
        if(controller.getRawButton(button)){
          whileHeld.run();
        }
    }

    public static void whenPressed(GenericHID controller, int button, Runnable onPress){
        if(controller.getRawButtonPressed(button)){
            onPress.run();
        }
    }

    public static void whenReleased(GenericHID controller, int button, Runnable onRelease){
        if(controller.getRawButtonReleased(button)){
            onRelease.run();
        }
    }

    public static void whenTriggerPressed(boolean current, boolean last, Runnable onPress) {
        if (!last && current) {
            onPress.run();
        }
    }

    public static void whenTriggerReleased(boolean current, boolean last, Runnable onRelease) {
        if (last && !current) {
            onRelease.run();
        }
    }

    public static void whileTriggerHeld(boolean current,  Runnable whileHeld) {
        if (current) {
            whileHeld.run();
        }
    }

    public static void whenPressedAndReleased(GenericHID controller, int button, Runnable onPress, Runnable whenReleased){
        whenPressed(controller, button, onPress);
        whenReleased(controller, button, whenReleased);
    }
    

    public static void whenPressedHeldAndReleased(GenericHID controller, int button, Runnable onPress, Runnable whileHeld, Runnable whenReleased){
        whenPressed(controller, button, onPress);
        whileHeld(controller, button, whileHeld);
        whenReleased(controller, button, whenReleased);
    }

    public static void whenTriggerPressedAndReleased(boolean current, boolean last, Runnable whenPressed, Runnable whenRelesaed) {
        whenTriggerPressed(current, last, whenPressed);
        whenTriggerReleased(current, last, whenRelesaed);
    }


    public static void whenTriggerPressedHeldAndReleased(boolean current, boolean last, Runnable whenPressed, Runnable whileHeld, Runnable whenRelesaed) {
        whenTriggerPressed(current, last, whenPressed);
        whileTriggerHeld(current, whileHeld);
        whenTriggerReleased(current, last, whenRelesaed);
    }

    public static void whileHeldAndReleased(GenericHID controller, int button, Runnable whileHeld, Runnable whenReleased){
        whileHeld(controller, button, whileHeld);
        whenReleased(controller, button, whenReleased);
    }

    public static boolean getButton(GenericHID controller, int button){
        return controller.getRawButton(button);
    }

    public static double getDriverLeftYValue() {
        return driver.getRawAxis(1);
    }

    public static double getDriverLeftXValue() {
        return driver.getRawAxis(0);
    }

    public static double getDriverRightYValue() {
        return driver.getRawAxis(5);
    }

    public static double getDriverRightXValue() {
        return driver.getRawAxis(4);
    }

    public static boolean getDriverLeftTrigger() {
        return driver.getRawAxis(2) >= 0.2;
    }

    public static boolean getDriverRightTrigger() {
        return driver.getRawAxis(3) >= 0.2;
    }

    public static boolean getOverrideRightTrigger() {
        return overrideController.getRawAxis(3) >= 0.5;
    }

    public static double getOperatorLeftYValue() {
        return overrideController.getRawAxis(1);
    }

    public static double getOperatorLeftXValue() {
        return overrideController.getRawAxis(0);
    }

    public static boolean driverXpressed() {
        return getButton(driver, buttonX);
    }

    public static boolean driverYpressed() {
        return getButton(driver, buttonY);
    }

    public static boolean tryingToDrive() {
        return tryingToTranslate() || tryingToTurn();
    }
    
    public static boolean tryingToTranslate() {
        return (new Translation2d(getDriverLeftXValue(), getDriverLeftYValue())).getNorm() >= Drive.JOYSTICK_DRIVE_TOLERANCE;
    }

    public static boolean tryingToTurn() {
        return (Math.abs(getDriverRightXValue()) > Drive.JOYSTICK_TURN_TOLERANCE);
    }

    public static void flushOICache(){
        for(int i = 0; i <= 2;i++){
            GenericHID controller = new GenericHID(i);
            for(int j = 1; j <= 12; j++){
                controller.getRawButton(j);
            }
        }
    }

}