package frc.robot.util;

import static frc.robot.util.DashboardControlsTestUtil.DRIVE;
import static frc.robot.util.DashboardControlsTestUtil.FLYWHEEL;
import static frc.robot.util.DashboardControlsTestUtil.HOOD;
import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.subsystems.Hood.HoodConstants;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/**
 * Power-cycle persistence of the Elastic controls: values edited on the dashboard are saved, and
 * restored on the next boot instead of falling back to the code defaults.
 */
class DashboardControlsPersistenceTest {
  private static final double EPS = 1e-9;

  @TempDir Path tempDir;

  @AfterEach
  void resetDashboard() {
    DashboardControlsTestUtil.resetDefaults();
    DashboardControls.setSettingsFileForTest(null);
  }

  @Test
  void rebootRestoresValuesEditedOnTheDashboard() {
    DashboardSettingsFile store = new DashboardSettingsFile(tempDir.resolve("settings.txt"));
    DashboardControls.setSettingsFileForTest(store);

    // "Elastic edits": dashboard writes, then the loop persists what changed.
    DashboardControlsTestUtil.set(DRIVE, 20.0);
    DashboardControlsTestUtil.set(HOOD, 25.0);
    DashboardControlsTestUtil.setFullKey(DashboardControls.LED_BRIGHTNESS_KEY, 42.0);
    DashboardControls.persistIfChanged();
    store.saveNow();

    // "Power cycle": a fresh store over the same file, then boot-time restore.
    DashboardControls.setSettingsFileForTest(
        new DashboardSettingsFile(tempDir.resolve("settings.txt")));
    DashboardControls.applySavedSettings();
    DashboardControlsTestUtil.refresh();

    assertEquals(0.2, DashboardControls.getDriveScale(), EPS);
    assertEquals(
        HoodConstants.MinAngleDeg + 0.25 * (HoodConstants.MaxAngleDeg - HoodConstants.MinAngleDeg),
        DashboardControls.getHoodTargetDeg(),
        EPS);
    assertEquals(42.0, DashboardControls.getLedBrightnessPercent(), EPS);
    // Keys the operator never moved come back at their defaults.
    assertEquals(1.0, DashboardControls.getTurnScale(), EPS);
    assertEquals(1.0, DashboardControls.getFlywheelScale(), EPS);
  }

  @Test
  void missingFileKeepsCodeDefaults() {
    DashboardControls.setSettingsFileForTest(
        new DashboardSettingsFile(tempDir.resolve("settings.txt")));
    DashboardControls.applySavedSettings();
    DashboardControlsTestUtil.refresh();

    assertEquals(1.0, DashboardControls.getDriveScale(), EPS);
    assertEquals(1.0, DashboardControls.getTurnScale(), EPS);
    assertEquals(1.0, DashboardControls.getFlywheelScale(), EPS);
    assertEquals(100.0, DashboardControls.getLedBrightnessPercent(), EPS);
    assertEquals(HoodConstants.MinAngleDeg, DashboardControls.getHoodTargetDeg(), EPS);
  }

  @Test
  void restoredValuesAreClampedBackIntoRange() throws Exception {
    Path file = tempDir.resolve("settings.txt");
    Files.writeString(
        file,
        DashboardControls.HOOD_KEY
            + "=140.0\n"
            + DashboardControls.DRIVE_KEY
            + "=NaN\n"
            + DashboardControls.TURN_KEY
            + "=-30.0\n");
    DashboardControls.setSettingsFileForTest(new DashboardSettingsFile(file));
    DashboardControls.applySavedSettings();
    DashboardControlsTestUtil.refresh();

    assertEquals(HoodConstants.MaxAngleDeg, DashboardControls.getHoodTargetDeg(), EPS);
    assertEquals(1.0, DashboardControls.getDriveScale(), EPS);
    assertEquals(0.0, DashboardControls.getTurnScale(), EPS);
    assertEquals(1.0, DashboardControls.getFlywheelScale(), EPS);
  }

  @Test
  void flywheelDpadsNudgeIsPersistedToo() {
    DashboardSettingsFile store = new DashboardSettingsFile(tempDir.resolve("settings.txt"));
    DashboardControls.setSettingsFileForTest(store);

    DashboardControlsTestUtil.set(FLYWHEEL, 60.0);
    DashboardControls.adjustFlywheelPercent(-10.0);
    DashboardControlsTestUtil.refresh();
    DashboardControls.persistIfChanged();
    store.saveNow();

    DashboardControls.setSettingsFileForTest(
        new DashboardSettingsFile(tempDir.resolve("settings.txt")));
    DashboardControls.applySavedSettings();
    DashboardControlsTestUtil.refresh();

    assertEquals(0.5, DashboardControls.getFlywheelScale(), EPS);
    // The other controls were still at defaults when the save happened.
    assertEquals(1.0, DashboardControls.getDriveScale(), EPS);
  }
}
