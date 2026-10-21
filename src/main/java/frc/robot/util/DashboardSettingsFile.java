package frc.robot.util;

import edu.wpi.first.wpilibj.RobotBase;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.AtomicMoveNotSupportedException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

/**
 * Persists the Elastic dashboard settings (numeric percentages and string selections such as the
 * LED pattern) across robot reboots and power cycles, so an operator-set value (e.g. 20% drive
 * speed) is still 20% the next time the robot is turned on.
 *
 * <p>Values are stored in a plain-text {@code key=value} file (one full NT key per line, value as
 * its string form). Writes are debounced and run on a background daemon thread so the real-time
 * loop never touches disk, and use a write-to-temp-then-rename so a power loss mid-save cannot
 * corrupt the previous file.
 *
 * <p>The file is treated as untrusted input: a missing file, an oversized file, or any unreadable
 * line is skipped with a console warning and falls back to the code default for that key — never an
 * exception. Values are re-sanitized by the caller after loading; numeric reads additionally fall
 * back when the stored value is not a finite number.
 *
 * <p>Location: roboRIO {@code /home/lvuser/dashboard-settings.txt} (survives reboots, redeploys and
 * power cycles; only a re-image wipes it), simulation {@code build/dashboard-settings.txt}.
 */
final class DashboardSettingsFile {
  private static final long SAVE_DELAY_MS = 400;
  private static final long MAX_FILE_BYTES = 64 * 1024;

  private final Path file;
  private final ScheduledExecutorService executor =
      Executors.newSingleThreadScheduledExecutor(
          runnable -> {
            Thread thread = new Thread(runnable, "DashboardSettingsSaver");
            thread.setDaemon(true);
            return thread;
          });
  private final Map<String, String> persisted = new LinkedHashMap<>();
  private final Map<String, String> pending = new LinkedHashMap<>();
  private boolean loaded;
  private ScheduledFuture<?> pendingSave;

  /** Creates a store backed by an explicit file (used by tests). */
  DashboardSettingsFile(Path file) {
    this.file = file;
  }

  /** Creates the store for the current environment: roboRIO home dir, or build/ in simulation. */
  static DashboardSettingsFile createDefault() {
    return new DashboardSettingsFile(
        RobotBase.isReal()
            ? Path.of("/home/lvuser/dashboard-settings.txt")
            : Path.of("build", "dashboard-settings.txt"));
  }

  /** Reads the file once; later calls are no-ops. */
  private synchronized void ensureLoaded() {
    if (!loaded) {
      persisted.putAll(readFile());
      loaded = true;
    }
  }

  /**
   * Returns the saved value of a numeric key, or {@code fallback} when the key is absent or its
   * stored form is not a finite number.
   */
  double getDouble(String key, double fallback) {
    String raw = getString(key, null);
    if (raw == null) {
      return fallback;
    }
    try {
      double value = Double.parseDouble(raw.trim());
      if (!Double.isFinite(value)) {
        throw new NumberFormatException("not finite");
      }
      return value;
    } catch (NumberFormatException e) {
      System.err.println(
          "[DashboardSettingsFile] Value of " + key + " is not a usable number: " + raw);
      return fallback;
    }
  }

  /** Returns the saved value of a string key, or {@code fallback} when the key is absent. */
  String getString(String key, String fallback) {
    ensureLoaded();
    synchronized (this) {
      String value = pending.containsKey(key) ? pending.get(key) : persisted.get(key);
      return value != null ? value : fallback;
    }
  }

  /** Records the latest numeric value of a key; see {@link #updateString(String, String)}. */
  void update(String key, double value) {
    updateString(key, Double.toString(value));
  }

  /**
   * Records the latest value of a key. A save is scheduled a short delay after the value last
   * changed (so a dragged Elastic slider produces one write, not hundreds). The comparison is
   * against the latest value seen this session — not the on-disk state — because the caller polls
   * every loop: a value that merely differs from disk (e.g. before the first save) must not keep
   * pushing the timer out, or the save would never fire.
   */
  synchronized void updateString(String key, String value) {
    // The file format is one line per entry; newlines would corrupt it.
    String sanitized = value.replace('\n', ' ').replace('\r', ' ').trim();
    String latest = pending.containsKey(key) ? pending.get(key) : persisted.get(key);
    if (sanitized.equals(latest)) {
      return;
    }
    pending.put(key, sanitized);
    if (pendingSave != null) {
      pendingSave.cancel(false);
    }
    pendingSave = executor.schedule(this::runSave, SAVE_DELAY_MS, TimeUnit.MILLISECONDS);
  }

  /** Writes pending changes immediately on the caller thread (used by tests). */
  synchronized void saveNow() {
    if (pendingSave != null) {
      pendingSave.cancel(false);
      pendingSave = null;
    }
    doSave();
  }

  private void runSave() {
    synchronized (this) {
      pendingSave = null;
    }
    doSave();
  }

  private synchronized void doSave() {
    if (pending.isEmpty()) {
      return;
    }
    Map<String, String> toWrite = new LinkedHashMap<>(persisted);
    toWrite.putAll(pending);
    try {
      writeFile(toWrite);
      persisted.putAll(toWrite);
      loaded = true;
      // Keep only values that changed again while the write was in flight.
      pending.entrySet().removeIf(e -> toWrite.get(e.getKey()).equals(e.getValue()));
    } catch (IOException e) {
      // Keep pending so the next change retries; no automatic retry loop.
      System.err.println("[DashboardSettingsFile] Could not save " + file + ": " + e.getMessage());
    }
  }

  private void writeFile(Map<String, String> values) throws IOException {
    Path parent = file.getParent();
    if (parent != null) {
      Files.createDirectories(parent);
    }
    List<String> lines = new ArrayList<>();
    lines.add("# Elastic dashboard settings, auto-saved by the robot program.");
    lines.add("# One <NT key>=<value> per line. Edit only while the robot program is off.");
    for (Map.Entry<String, String> entry : values.entrySet()) {
      lines.add(entry.getKey() + "=" + entry.getValue());
    }
    Path temp = file.resolveSibling(file.getFileName() + ".tmp");
    Files.write(temp, lines, StandardCharsets.UTF_8);
    try {
      Files.move(temp, file, StandardCopyOption.REPLACE_EXISTING, StandardCopyOption.ATOMIC_MOVE);
    } catch (AtomicMoveNotSupportedException e) {
      Files.move(temp, file, StandardCopyOption.REPLACE_EXISTING);
    }
  }

  private Map<String, String> readFile() {
    Map<String, String> result = new LinkedHashMap<>();
    if (!Files.isRegularFile(file)) {
      return result;
    }
    try {
      if (Files.size(file) > MAX_FILE_BYTES) {
        System.err.println(
            "[DashboardSettingsFile] " + file + " is unexpectedly large; using code defaults");
        return result;
      }
      for (String rawLine : Files.readAllLines(file, StandardCharsets.UTF_8)) {
        String line = rawLine.trim();
        if (line.isEmpty() || line.startsWith("#")) {
          continue;
        }
        int separator = line.indexOf('=');
        if (separator <= 0) {
          System.err.println("[DashboardSettingsFile] Skipping malformed line: " + line);
          continue;
        }
        String key = line.substring(0, separator).trim();
        String value = line.substring(separator + 1).trim();
        result.put(key, value);
      }
    } catch (IOException e) {
      System.err.println("[DashboardSettingsFile] Could not read " + file + ": " + e.getMessage());
    }
    return result;
  }
}
