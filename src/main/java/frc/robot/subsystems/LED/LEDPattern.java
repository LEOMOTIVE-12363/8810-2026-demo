package frc.robot.subsystems.LED;

/**
 * Built-in effects for the LED strip, selectable from the Elastic "LED Pattern" chooser.
 *
 * <p>To add a new effect: add an enum constant here (its {@code dashboardName} shows up on the
 * chooser automatically) and a matching case in {@link LED#periodic()}.
 */
public enum LEDPattern {
  OFF("Off"),
  SOLID("Solid"),
  RAINBOW("Rainbow"),
  CHASE("Chase"),
  BREATHE("Breathe");

  private final String dashboardName;

  LEDPattern(String dashboardName) {
    this.dashboardName = dashboardName;
  }

  /** Label shown for this pattern on the dashboard chooser. */
  public String getDashboardName() {
    return dashboardName;
  }
}
