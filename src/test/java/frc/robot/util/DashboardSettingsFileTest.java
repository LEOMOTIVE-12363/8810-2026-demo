package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Map;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class DashboardSettingsFileTest {
  private static final String DRIVE_KEY = "/SmartDashboard/Drive Speed Limit (%)";
  private static final String TURN_KEY = "/SmartDashboard/Turn Speed Limit (%)";

  @TempDir Path tempDir;

  private Path settingsPath() {
    return tempDir.resolve("dashboard-settings.txt");
  }

  @Test
  void loadReturnsEmptyWhenNoFileExists() {
    DashboardSettingsFile store = new DashboardSettingsFile(settingsPath());
    assertTrue(store.load().isEmpty());
  }

  @Test
  void savedValuesRoundTripThroughTheFile() {
    DashboardSettingsFile store = new DashboardSettingsFile(settingsPath());
    store.update(DRIVE_KEY, 20.0);
    store.update(TURN_KEY, 87.5);
    store.saveNow();

    // A fresh instance (as after a reboot) reads the same values back.
    Map<String, Double> loaded = new DashboardSettingsFile(settingsPath()).load();
    assertEquals(20.0, loaded.get(DRIVE_KEY));
    assertEquals(87.5, loaded.get(TURN_KEY));
  }

  @Test
  void saveOverwritesPreviousValuesAndLeavesNoTempFile() throws IOException {
    DashboardSettingsFile store = new DashboardSettingsFile(settingsPath());
    store.update(DRIVE_KEY, 20.0);
    store.saveNow();
    store.update(DRIVE_KEY, 40.0);
    store.saveNow();

    assertEquals(40.0, new DashboardSettingsFile(settingsPath()).load().get(DRIVE_KEY));
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
    assertEquals(55.0, new DashboardSettingsFile(settingsPath()).load().get(DRIVE_KEY));
  }

  @Test
  void corruptLinesAreSkippedNotFatal() throws IOException {
    Files.writeString(
        settingsPath(),
        """
        garbage line without an equals sign
        # comment line
        /SmartDashboard/Drive Speed Limit (%)=notanumber
        /bogus/key=NaN
        /SmartDashboard/Turn Speed Limit (%)=33
        """);

    Map<String, Double> loaded = new DashboardSettingsFile(settingsPath()).load();
    assertEquals(1, loaded.size());
    assertEquals(33.0, loaded.get(TURN_KEY));
  }

  @Test
  void oversizedFileFallsBackToEmpty() throws IOException {
    Files.writeString(settingsPath(), "# " + "x".repeat(70_000) + "\n");

    assertTrue(new DashboardSettingsFile(settingsPath()).load().isEmpty());
  }
}
