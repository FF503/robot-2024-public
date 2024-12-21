// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frogforce503.lib.util;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import org.frogforce503.lib.drivers.CANMotor;
import org.frogforce503.lib.drivers.CANMotor.MotorControlMode;
import org.frogforce503.robot2024.planners.ShotPlanner;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** 
 * <p> This class is a thin wrapper around Shuffleboard & NetworkTables that makes writing output to Shuffleboard / NetworkTables easier. </p>
 * <p> This class handles Shuffleboard components, layouts, and more (including easy PIDF tuning). </p> 
 * <strong> Make sure to initalize your component / layout / other data before your update it. </strong>
 */
public class SBUtil {
    static HashMap<String, Pair<ShuffleboardLayout, NetworkTable>> layoutMap = new HashMap<>();
    static HashMap<String, HashMap<String, GenericEntry>> componentMap = new HashMap<>();
    static HashMap<String, HashMap<String, Supplier<Object>>> subsystemTiedSuppliers = new HashMap<>();
    
    private SBUtil() {}

    // -------------------------------------- COMPONENT INIT -------------------------------------- //

    @USE_SHUFFLEBOARD
    public static void initComponent(String tabName, String cName, Object value, int posX, int posY, int width, int height, BuiltInWidgets widget, Map<String, Object> properties) {
        GenericEntry c = Shuffleboard.getTab(tabName).add(cName, value).withPosition(posX, posY).withSize(width, height).withWidget(widget).withProperties(properties).getEntry();
        if (!componentMap.containsKey(tabName))
            componentMap.put(tabName, new HashMap<>());
        
        componentMap.get(tabName).put(cName, c);
    }

    @USE_SHUFFLEBOARD
    public static void initComponent(String tabName, String cName, Object value, int posX, int posY, int width, int height, BuiltInWidgets widget) {
        initComponent(tabName, cName, value, posX, posY, width, height, widget, null);
    }
    
    @USE_SHUFFLEBOARD
    public static void initComponent(String tabName, String cName, Object value, int posX, int posY, int width, int height, Map<String, Object> properties) {
        initComponent(tabName, cName, value, posX, posY, width, height, BuiltInWidgets.kTextView, properties);
    }

    @USE_SHUFFLEBOARD
    public static void initComponent(String tabName, String cName, Object value, int posX, int posY, int width, int height) {
        initComponent(tabName, cName, value, posX, posY, width, height, BuiltInWidgets.kTextView, null);
    }

    @USE_SHUFFLEBOARD
    public static void initComponent(String tabName, String cName, Object value, int posX, int posY) {
        initComponent(tabName, cName, value, posX, posY, 2, 1);
    }
    
    /**
     * Initializes a supplier-based component updating every time the tab / subsystem is updated
     * @param tabName Name of tab / subsystem
     * @param cName Name of component
     */
    @USE_OTHER_DASHBOARD
    public static void initComponent(String tabName, String cName, Supplier<Object> value) {
        initComponent(tabName, cName, value.get(), 0, 0); // put the initial value on NT
        
        if (!subsystemTiedSuppliers.containsKey(tabName))
            subsystemTiedSuppliers.put(tabName, new HashMap<>());
        
        subsystemTiedSuppliers.get(tabName).put(cName, value);
    }

    @USE_OTHER_DASHBOARD
    public static void initComponent(String tabName, String cName, Object defaultValue) {
        initComponent(tabName, cName, defaultValue, 0, 0);
    }

    @USE_OTHER_DASHBOARD
    public static void initComponent(String tabName, String cName, Object defaultValue, BuiltInWidgets widget) {
        initComponent(tabName, cName, defaultValue, 0, 0, 2, 1, widget);
    }

    // -------------------------------------- COMPONENT SETTERS -------------------------------------- //

    public static void setComponent(String tabName, String cName, Object o) {
        getComponentSB(tabName, cName).setValue(o);
    }

    // -------------------------------------- COMPONENT GETTERS -------------------------------------- //

    public static String getComponent(String tabName, String cName, String defaultValue) {
        return getComponentSB(tabName, cName).getString(defaultValue);
    }
    
    public static double getComponent(String tabName, String cName, double defaultValue) {
        return getComponentSB(tabName, cName).getDouble(defaultValue);
    }
    
    public static boolean getComponent(String tabName, String cName, boolean defaultValue) {
        return getComponentSB(tabName, cName).getBoolean(defaultValue);
    }

    private static GenericEntry getComponentSB(String tabName, String cName) {
        return componentMap.get(tabName).get(cName);
    }

    // -------------------------------------- LAYOUT INIT -------------------------------------- //

    @USE_SHUFFLEBOARD
    public static void initLayout(String tabName, String layoutName, BuiltInLayouts layoutType, int posX, int posY, int width, int height) {
        ShuffleboardLayout l = Shuffleboard.getTab(tabName).getLayout(layoutName, layoutType).withPosition(posX, posY).withSize(width, height);
        NetworkTable n = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable(tabName).getSubTable(layoutName);
        layoutMap.put(layoutName, new Pair<ShuffleboardLayout, NetworkTable>(l, n));
    }

    @USE_OTHER_DASHBOARD
    public static void initLayout(String tabName, String layoutName) {
        initLayout(tabName, layoutName, BuiltInLayouts.kList, 0, 0, 1, 1);
    }

    // -------------------------------------- LAYOUT COMPONENT ADDERS  -------------------------------------- //

    @USE_SHUFFLEBOARD
    public static void add(String layoutName, String cName, Object value, int cWidth, int cHeight, BuiltInWidgets widget, Map<String, Object> properties) {
        layoutMap.get(layoutName).getFirst().add(cName, value).withSize(cWidth, cHeight).withWidget(widget).withProperties(properties);
    }

    @USE_SHUFFLEBOARD
    public static void add(String layoutName, String cName, Object value, int cWidth, int cHeight, BuiltInWidgets widget) {
        add(layoutName, cName, value, cWidth, cHeight, widget, null);
    }
    
    @USE_SHUFFLEBOARD
    public static void add(String layoutName, String cName, Object value, int cWidth, int cHeight, Map<String, Object> properties) {
        add(layoutName, cName, value, cWidth, cHeight, BuiltInWidgets.kTextView, properties);
    }

    @USE_SHUFFLEBOARD
    public static void add(String layoutName, String cName, Object value, int cWidth, int cHeight) {
        add(layoutName, cName, value, cWidth, cHeight, BuiltInWidgets.kTextView, null);
    }

    @USE_OTHER_DASHBOARD
    public static void add(String layoutName, String cName, Object value) {
        add(layoutName, cName, value, 1, 1);
    }

    // -------------------------------------- LAYOUT COMPONENT SETTERS -------------------------------------- //

    public static boolean set(String layoutName, String entry, Object entryValue) {
        return getLayout(layoutName).getEntry(entry).setValue(entryValue);
    }

    // -------------------------------------- LAYOUT COMPONENT GETTERS -------------------------------------- //

    public static String get(String layoutName, String entry, String entryValue) {
        return getLayout(layoutName).getEntry(entry).getString(entryValue);
    }

    public static double get(String layoutName, String entry, double entryValue) {
        return getLayout(layoutName).getEntry(entry).getDouble(entryValue);
    }

    public static long get(String layoutName, String entry, int entryValue) {
        return getLayout(layoutName).getEntry(entry).getInteger(entryValue);
    }

    public static boolean get(String layoutName, String entry, boolean entryValue) {
        return getLayout(layoutName).getEntry(entry).getBoolean(entryValue);
    }

    private static NetworkTable getLayout(String layoutName) {
        return layoutMap.get(layoutName).getSecond();
    }

    // -------------------------------------- MISCELLANEOUS -------------------------------------- //

    public static void initPIDF(String tabName, String subsystemName, CANMotor a) {
        initLayout(tabName, subsystemName + "/PIDs");

        add(subsystemName + "/PIDs", "P", a.getP());
        add(subsystemName + "/PIDs", "I", a.getI());
        add(subsystemName + "/PIDs", "D", a.getD());
        add(subsystemName + "/PIDs", "FF", a.getF());
    }

    public static void updatePIDF(String subsystemName, CANMotor a) {
        double sbP = get(subsystemName + "/PIDs", "P", 0.0);
        double sbI = get(subsystemName + "/PIDs", "I", 0.0);
        double sbD = get(subsystemName + "/PIDs", "D", 0.0);
        double sbF = get(subsystemName + "/PIDs", "FF", 0.0);
        
        if (a.getP() != sbP || a.getI() != sbI || a.getD() != sbD || a.getF() != sbF) {
            a.setPIDF(0, sbP, sbI, sbD, sbF);
        }
    }

    public static void initSubsystemData(String tabName, String subsystemName, CANMotor motor, MotorControlMode controlMode) {
        subsystemName += " Motor";

        initPIDF(tabName, subsystemName, motor);

        initComponent(tabName, subsystemName + "/Tuning PIDs?", false, BuiltInWidgets.kToggleSwitch);
        initComponent(tabName, subsystemName + "/In Coast?", false, BuiltInWidgets.kToggleSwitch);
        initComponent(tabName, subsystemName + "/" + controlMode.toString(), motor.getOutputFromMode(controlMode));
        initComponent(tabName, subsystemName + "/Current (Amps)", motor.getSparkMax().getOutputCurrent());
        initComponent(tabName, subsystemName + "/Temperature (Celsius)", motor.getSparkMax().getMotorTemperature());
    }

    public static void updateSubsytemData(String tabName, String subsystemName, CANMotor motor, MotorControlMode controlMode) {
        subsystemName += " Motor";

        if (getComponent(tabName, subsystemName + "/Tuning PIDs?", true)) {
            updatePIDF(subsystemName, motor);
        }

        setComponent(tabName, subsystemName + "/" + controlMode.toString(), motor.getOutputFromMode(controlMode));
        setComponent(tabName, subsystemName + "/Current (Amps)", motor.getSparkMax().getOutputCurrent());
        setComponent(tabName, subsystemName + "/Temperature (Celsius)", motor.getSparkMax().getMotorTemperature());

        if (subsystemTiedSuppliers.containsKey(tabName)) {
            for (Map.Entry<String, Supplier<Object>> pair : subsystemTiedSuppliers.get(tabName).entrySet()) {
                setComponent(tabName, pair.getKey(), pair.getValue().get());
            }
        }
    }

    static ArrayList<double[]> currentWorkingShotmap = new ArrayList<>();

    public static void initShotmapHelper() {
        initComponent("ShotPlanner", "Tuning ShotMap?", false);
        initComponent("ShotPlanner", "Distance To Goal", 0.0);
        initComponent("ShotPlanner", "Wrist Angle ShotMap", 0.0);
        initComponent("ShotPlanner", "Arm Angle ShotMap", 0.0);
        initComponent("ShotPlanner", "Add To ShotMap", false);
        initComponent("ShotPlanner", "Load From Existing ShotMap", false);
        initComponent("ShotPlanner", "Load From Working ShotMap", false);
        initComponent("ShotPlanner", "Current ShotMap", "{}");
    }

    public static void updateShotmapHelper() {
        setComponent("ShotPlanner", "Distance To Goal", ShotPlanner.getInstance().getDistanceTarget());

        boolean isTuning = getComponent("ShotPlanner", "Tuning ShotMap?", true);
        ShotPlanner.getInstance().setTuning(isTuning);
        
        if (isTuning) {
            if (getComponent("ShotPlanner", "Add To ShotMap", true)) {
                System.out.println("ADDED");

                currentWorkingShotmap.add(
                    new double[] {
                        ShotPlanner.getInstance().getDistanceTarget(),
                        getComponent("ShotPlanner", "Arm Angle ShotMap", 0.0),
                        getComponent("ShotPlanner", "Wrist Angle ShotMap", 0.0)
                    }
                );
                
                String outputMap = "";
                
                for (double[] set : currentWorkingShotmap) {
                    outputMap += "{" + set[0] + ", " + set[1] + ", " + set[2] + "},";
                }

                setComponent("ShotPlanner", "Current ShotMap", outputMap);
                setComponent("ShotPlanner", "Add To ShotMap", false);
            }

            if (getComponent("ShotPlanner", "Load From Existing ShotMap", true)) {
                setComponent("ShotPlanner", "Wrist Angle ShotMap", ShotPlanner.getInstance().getWristAngleForDistance(ShotPlanner.getInstance().getDistanceTarget()));
                setComponent("ShotPlanner", "Arm Angle ShotMap", ShotPlanner.getInstance().getArmAngleForDistance(ShotPlanner.getInstance().getDistanceTarget()));

                setComponent("ShotPlanner", "Load From Existing ShotMap", false);
            }

            if (getComponent("ShotPlanner", "Load From Working ShotMap", true)) {
                InterpolatingDoubleTreeMap treeMap = new InterpolatingDoubleTreeMap();
    
                for (double[] set : currentWorkingShotmap) {
                    treeMap.put(set[0], set[1]);
                }

                setComponent("ShotPlanner", "Wrist Angle ShotMap", treeMap.get(ShotPlanner.getInstance().getDistanceTarget()));
                setComponent("ShotPlanner", "Load From Existing ShotMap", false);
            }
        }
    }




    // --------------------------------------------- RELEVANT LABELS --------------------------------------------- //

    /**
     * Indicates a method is BEST used for {@linkplain Shuffleboard}
     */
    @Target(ElementType.METHOD)
    @Retention(RetentionPolicy.SOURCE)
    public @interface USE_SHUFFLEBOARD {}

    /**
     * <p> Indicates a method is BEST used for dashboards other than {@linkplain Shuffleboard} </p>
     * <strong> Mostly used for AdvantageScope / {@linkplain SmartDashboard} values </strong>
     */
    @Target(ElementType.METHOD)
    @Retention(RetentionPolicy.SOURCE)
    public @interface USE_OTHER_DASHBOARD {}
}