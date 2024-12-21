// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.frogforce503.lib.util;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import java.util.Arrays;
import java.util.HashMap;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;
import org.littletonrobotics.junction.networktables.LoggedDashboardString;

/** A string chooser for the dashboard where the options can be changed on-the-fly. */
public class SwitchableChooser<E> implements LoggedDashboardInput {
  private static final String placeholder = "<NA>";

  private String[] internalOptions = new String[] {placeholder};
  private HashMap<String, E> optionMap = new HashMap<>();
  private String active = placeholder;

  private final StringPublisher namePublisher;
  private final StringPublisher typePublisher;
  private final StringArrayPublisher optionsPublisher;
  private final StringPublisher defaultPublisher;
  private final StringPublisher activePublisher;
  private final StringPublisher selectedPublisher;
  private final LoggedDashboardString selectedInput;

  public SwitchableChooser(String name) {
    var table = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable(name);
    namePublisher = table.getStringTopic(".name").publish();
    typePublisher = table.getStringTopic(".type").publish();
    optionsPublisher = table.getStringArrayTopic("options").publish();
    defaultPublisher = table.getStringTopic("default").publish();
    activePublisher = table.getStringTopic("active").publish();
    selectedPublisher = table.getStringTopic("selected").publish();
    selectedInput = new LoggedDashboardString(name + "/selected");
    Logger.registerDashboardInput(this);

    namePublisher.set(name);
    typePublisher.set("String Chooser");
    optionsPublisher.set(this.internalOptions);
    defaultPublisher.set(this.internalOptions[0]);
    activePublisher.set(this.internalOptions[0]);
    selectedPublisher.set(this.internalOptions[0]);
  }

  /** Updates the set of available options. */
  public void setOptions(E[] options) {
    if (Arrays.equals(options, this.optionMap.keySet().toArray())) {
      return;
    }

    this.optionMap.clear();
    for (E option : options) {
      this.optionMap.put(option.toString(), option);
    }

    this.internalOptions = options.length == 0 ? new String[] {placeholder} : this.optionMap.keySet().toArray(String[]::new);
    optionsPublisher.set(this.internalOptions);
    periodic();
  }

  /** Returns the selected option. */
  public E get() {
    return active == placeholder ? null : this.optionMap.get(active);
  }

  public void periodic() {
    String selected = selectedInput.get();
    active = null;
    for (String option : internalOptions) {
      if (!option.equals(placeholder) && option.equals(selected)) {
        active = option;
      }
    }
    if (active == null) {
      active = internalOptions[0];
      selectedPublisher.set(active);
    }
    defaultPublisher.set(active);
    activePublisher.set(active);
  }
}