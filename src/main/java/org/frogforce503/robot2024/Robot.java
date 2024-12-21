// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frogforce503.robot2024;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.frogforce503.robot2024.subsystems.DriverFeedback;
import org.frogforce503.robot2024.subsystems.NoteDetector;
import org.frogforce503.robot2024.subsystems.NoteDetector.SORTING_MODE;
import org.frogforce503.robot2024.subsystems.Wrist.Goal;
import org.frogforce503.robot2024.subsystems.sim.Visualizer;
import org.frogforce503.robot2024.fields.FieldConfig;
import org.frogforce503.robot2024.fields.FieldConfig.VENUE;
import org.frogforce503.robot2024.hardware.RobotHardware;
import org.frogforce503.robot2024.planners.ShotPlanner;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  public static RobotHardware bot;

  private final NoteDetector mNoteDetector;
  /*
   * Robot Constructor 
   */
  public Robot() {
    RobotStatus.getInstance().setCurrentRobot(RobotStatus.Bot.CompBot);
    bot = RobotHardware.getInstance();
    mNoteDetector = NoteDetector.getInstance();
  }
 
 
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Logger.recordMetadata("ProjectName", "FF2024_" + RobotStatus.getInstance().getCurrentRobot().name().toUpperCase()); // Set a metadata value
    

    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables

    if (isReal()) {
        // Logger.addDataReceiver(new WPILOGWriter()); 
    } else {
        setUseTiming(false); // Run as fast as possible
        // String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        // Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        // Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    FieldConfig.getInstance().setVenue(VENUE.SHOP);

    RobotContainer.init();
    DriverFeedback.getInstance().setToDefault();
    Logger.start();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    Visualizer.getInstance().update();

    SmartDashboard.putData(CommandScheduler.getInstance());
    RobotContainer.shotplanner.update();
  }

  @Override
  public void autonomousInit() {
    RobotContainer.autoChooser.startAuto();
    RobotContainer.wrist.currentGoal = Goal.AIMING;
    RobotContainer.climbStep0();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    RobotContainer.autoChooser.cleanup();
    RobotContainer.noteDetector.setSortingMode(SORTING_MODE.CENTERMOST);
    ShotPlanner.getInstance().cancelHint();
    RobotContainer.shotplanner.cancelHint();
  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("CLIMB STEP", RobotContainer.climbStep);

    RobotContainer.periodic();
  }

  @Override
  public void disabledInit() {
    RobotContainer.drive.brake();
  }

  @Override
  public void disabledPeriodic() {
    RobotContainer.autoChooser.periodic();
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}