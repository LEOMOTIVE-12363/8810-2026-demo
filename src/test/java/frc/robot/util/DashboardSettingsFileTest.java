package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class DashboardSettingsFileTest {
  private static final String DRIVE_KEY = "/SmartDashboard/Drive Speed Limit (%)";
  private static final String TURN_KEY = "/SmartDashboard/Turn Speed Limit (%)";
  private static final String PATTERN_KEY = "/SmartDashboard/LED Pattern";

  @TempDir Path tempDir;

  private Path settingsPath() {
    return tempDir.resolve("dashboard-settings.txt");
  }

  @Test
  void missingFileFallsBackForEachKey() {
    DashboardSettingsFile store = new DashboardSettingsFile(settingsPath());
    assertEquals(100.0, store.getDouble(DRIVE_KEY, 100.0));
    assertEquals("Rainbow", store.getString(PATTERN_KEY, "Rainbow"));
  }

  @Test
  void savedNumbersRoundTripThroughTheFile() {
    DashboardSettingsFile store = new DashboardSettingsFile(settingsPath());
    store.update(DRIVE_KEY, 20.0);
    store.update(TURN_KEY, 87.5);
    store.saveNow();

    // A fresh instance (as after a reboot) reads the same values back.
    DashboardSettingsFile rebooted = new DashboardSettingsFile(settingsPath());
    assertEquals(20.0, rebooted.getDouble(DRIVE_KEY, -1.0));
    assertEquals(87.5, rebooted.getDouble(TURN_KEY, -1.0));
  }

  @Test
  void savedStringsRoundTripThroughTheFile() {
    DashboardSettingsFile store = new DashboardSettingsFile(settingsPath());
    store.updateString(PATTERN_KEY, "Chase");
    store.update(DRIVE_KEY, 30.0);
    store.saveNow();

    DashboardSettingsFile rebooted = new DashboardSettingsFile(settingsPath());
    assertEquals("Chase", rebooted.getString(PATTERN_KEY, "Rainbow"));
    assertEquals(30.0, rebooted.getDouble(DRIVE_KEY, -1.0));
  }

  @Test
  void saveOverwritesPreviousValuesAndLeavesNoTempFile() throws IOException {
    DashboardSettingsFile store = new DashboardSettingsFile(settingsPath());
    store.update(DRIVE_KEY, 20.0);
    store.saveNow();
    store.update(DRIVE_KEY, 40.0);
    store.saveNow();

    assertEquals(40.0, new DashboardSettingsFile(settingsPath()).getDouble(DRIVE_KEY, -1.0));
    assertFalse(Files.exists(settingsPath().resolveSibling("dashboard-settings.txt.tmp")));
  }

  @Test
  void debouncedUpdateWritesTheFileWithoutSaveNow() throws Exception {
    DashboardSettingsFile store = new DashboardSettingsFile(settingsPath());
    store.update(DRIVE_KEY, 55.0);

    // The debounced save fires on its own; poll so a slow machine cannot flake the test.
    long deadline = System.currentTimeMillis() + 5000;
    while (!Files.exists(settingsPath()) && System.currentTimeMillis() < deadline) {
      Thread.sleep(50);
    }
    assertTrue(Files.exists(settingsPath()), "debounced save should have written the file");
    assertEquals(55.0, new DashboardSettingsFile(settingsPath()).getDouble(DRIVE_KEY, -1.0));
  }

  @Test
  void repeatedUnchangedUpdatesStillSave() throws Exception {
    DashboardSettingsFile store = new DashboardSettingsFile(settingsPath());

    // The robot loop polls every cycle with the same value. The save must still fire once and
    // stop: comparing against the on-disk state instead of the latest seen value used to defer
    // the timer on every poll, so nothing was ever written.
    long deadline = System.currentTimeMillis() + 5000;
    while (!Files.exists(settingsPath()) && System.currentTimeMillis() < deadline) {
      store.update(DRIVE_KEY, 30.0);
      Thread.sleep(20); // faster than the save delay, like the 50 Hz robot loop
    }
    assertTrue(Files.exists(settingsPath()), "save should fire despite unchanged repeated polls");
    assertEquals(30.0, new DashboardSettingsFile(settingsPath()).getDouble(DRIVE_KEY, -1.0));
  }

  @Test
  void malformedLinesAreSkippedAndBadNumbersFallBack() throws IOException {
    Files.writeString(
        settingsPath(),
        """
        garbage line without an equals sign
        # comment line
        /SmartDashboard/Drive Speed Limit (%)=notanumber
        /bogus/key=NaN
        /SmartDashboard/Turn Speed Limit (%)=33
        /SmartDashboard/LED Pattern=Solid
        """);

    DashboardSettingsFile store = new DashboardSettingsFile(settingsPath());
    assertEquals(100.0, store.getDouble(DRIVE_KEY, 100.0)); // unparsable -> fallback
    assertEquals(33.0, store.getDouble(TURN_KEY, -1.0)); // good line still loads
    assertEquals("Solid", store.getString(PATTERN_KEY, "Rainbow")); // strings coexist
  }

  @Test
  void oversizedFileFallsBackToEmpty() throws IOException {
    Files.writeString(
        settingsPath(), "/SmartDashboard/Drive Speed Limit (%)=33\n# " + "x".repeat(70_000) + "\n");

    DashboardSettingsFile store = new DashboardSettingsFile(settingsPath());
    assertEquals(100.0, store.getDouble(DRIVE_KEY, 100.0));
    assertEquals("Rainbow", store.getString(PATTERN_KEY, "Rainbow"));
  }
}
