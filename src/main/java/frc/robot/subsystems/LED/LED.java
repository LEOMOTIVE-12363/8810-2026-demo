package frc.robot.subsystems.LED;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.DashboardControls;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Addressable LED strip. The active effect comes from the Elastic "LED Pattern" chooser and global
 * brightness from the "/Tuning/LED Brightness (%)" slider (default 100%) — both power-cycle
 * persistent like the other Elastic settings. Robot-state-driven effects (alliance color, game
 * piece, …) will layer on top of this chooser later.
 */
public class LED extends SubsystemBase {
  public static final LED mInstance = new LED();

  /** Settings-store key of the chooser; its NT entry is "/SmartDashboard/LED Pattern". */
  public static final String PATTERN_SETTING_KEY = "/SmartDashboard/LED Pattern";

  private static final int MAX_VALUE = 255;

  private final LoggedDashboardChooser<LEDPattern> patternChooser =
      new LoggedDashboardChooser<>("LED Pattern");

  private final LEDIO io;
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LEDConstants.LENGTH_PIXELS);

  private LED() {
    io =
        switch (Constants.currentMode) {
          case REAL, SIM -> new LEDIOReal(LEDConstants.PWM_PORT, LEDConstants.LENGTH_PIXELS);
          case REPLAY -> new LEDIO() {};
        };

    patternChooser.addDefaultOption(LEDPattern.RAINBOW.getDashboardName(), LEDPattern.RAINBOW);
    for (LEDPattern pattern : LEDPattern.values()) {
      if (pattern != LEDPattern.RAINBOW) {
        patternChooser.addOption(pattern.getDashboardName(), pattern);
      }
    }

    restoreSavedPattern();
  }

  /**
   * Re-selects the pattern saved by the previous run, exactly as if the operator had picked it on
   * the dashboard (writes the chooser's "selected" NT entry, which Elastic drives the same way). An
   * unknown saved name (e.g. left over from an older build) is ignored, keeping the Rainbow
   * default.
   */
  private void restoreSavedPattern() {
    if (Constants.currentMode == Constants.Mode.REPLAY) {
      return;
    }
    String saved = DashboardControls.getSavedString(PATTERN_SETTING_KEY, null);
    if (saved == null) {
      return;
    }
    for (LEDPattern pattern : LEDPattern.values()) {
      if (pattern.getDashboardName().equals(saved)) {
        NetworkTableInstance.getDefault()
            .getEntry(PATTERN_SETTING_KEY + "/selected")
            .setString(saved);
        return;
      }
    }
    System.err.println("[LED] Unknown saved pattern \"" + saved + "\"; keeping the default");
  }

  @Override
  public void periodic() {
    LEDPattern pattern = patternChooser.get();
    if (pattern == null) {
      pattern = LEDPattern.OFF;
    }
    DashboardControls.persistStringIfChanged(PATTERN_SETTING_KEY, pattern.getDashboardName());
    double scale = DashboardControls.getLedBrightnessPercent() / 100.0 * MAX_VALUE;
    double time = Timer.getFPGATimestamp();
    int length = buffer.getLength();

    switch (pattern) {
      case OFF -> {
        for (int i = 0; i < length; i++) {
          buffer.setHSV(i, 0, 0, 0);
        }
      }
      case SOLID -> {
        for (int i = 0; i < length; i++) {
          buffer.setHSV(i, LEDConstants.SOLID_HUE, MAX_VALUE, (int) scale);
        }
      }
      case RAINBOW -> {
        int offset = (int) Math.round(time / LEDConstants.RAINBOW_FULL_CYCLE_SEC * 180.0) % 180;
        for (int i = 0; i < length; i++) {
          buffer.setHSV(i, (i * 180 / length + offset) % 180, MAX_VALUE, (int) scale);
        }
      }
      case CHASE -> {
        // Comet with CHASE_TAIL_PIXELS fade trailing its head, wrapping around the strip.
        double head =
            (time * LEDConstants.CHASE_SPEED_PIXELS_PER_SEC)
                % (length + LEDConstants.CHASE_TAIL_PIXELS);
        for (int i = 0; i < length; i++) {
          double distanceBehindHead = head - i;
          int value = 0;
          if (distanceBehindHead >= 0 && distanceBehindHead < LEDConstants.CHASE_TAIL_PIXELS) {
            value = (int) (scale * (1.0 - distanceBehindHead / LEDConstants.CHASE_TAIL_PIXELS));
          }
          buffer.setHSV(i, LEDConstants.CHASE_HUE, MAX_VALUE, value);
        }
      }
      case BREATHE -> {
        int value =
            (int)
                (scale
                    * (0.5 - 0.5 * Math.cos(2 * Math.PI * time / LEDConstants.BREATHE_PERIOD_SEC)));
        for (int i = 0; i < length; i++) {
          buffer.setHSV(i, LEDConstants.BREATHE_HUE, MAX_VALUE, value);
        }
      }
    }

    io.writeBuffer(buffer);
    Logger.recordOutput("LED/Pattern", pattern.toString());
    Logger.recordOutput("LED/BrightnessPercent", DashboardControls.getLedBrightnessPercent());
  }
}
