package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drum.Drum;
import frc.robot.subsystems.Drum.DrumConstants;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Operator-dashboard status: which mechanism action is running, whether feeding is allowed, and the
 * human-readable reason when not.
 *
 * <p>The status comes from command heartbeats ({@link #report}) plus the shared {@link
 * ShotReadiness} evaluation — it reports what the mechanism is actually commanded to do, never
 * guesses from buttons. Reports expire {@value #REPORT_EXPIRY_SEC} seconds after the last
 * heartbeat, so releasing a key, a command interruption, or a disable returns the panel to Idle on
 * its own — no stale status can pin.
 *
 * <p>The display publishes at 10 Hz from {@code Robot}'s dashboard-publish block via {@link
 * #publish} while control decisions ({@link #update}, {@link #feedAllowed}) run every loop. All
 * time comes from {@link Logger#getTimestamp()} (microseconds, replay-deterministic) unless a test
 * clock is injected via {@link #setClockForTest}; {@link #publish} writes nothing in replay.
 */
public final class OperatorStatus {
  /**
   * What a command reports while it runs. The effective action is the highest-priority fresh one
   * (see {@link #currentAction()}); IDLE is the no-fresh-reports fallback, not normally reported.
   */
  public enum Action {
    IDLE,
    INTAKING,
    SPINNING_UP,
    FEEDING,
    WAITING_RECOVERY,
    CLEARING_JAM,
    REVERSING_LOCAL,
    REVERSING_FULL
  }

  /** Seconds a report stays fresh after its last heartbeat. */
  private static final double REPORT_EXPIRY_SEC = 0.15;

  // Effective-action priority, highest first: eject/unjam handling outranks any feeding, feeding
  // outranks preparation. IDLE is not listed; it is the fallback when nothing is fresh.
  private static final Action[] PRIORITY = {
    Action.REVERSING_FULL,
    Action.REVERSING_LOCAL,
    Action.CLEARING_JAM,
    Action.WAITING_RECOVERY,
    Action.FEEDING,
    Action.SPINNING_UP,
    Action.INTAKING
  };

  // Last heartbeat timestamp (seconds) per Action, indexed by ordinal; NEGATIVE_INFINITY means
  // never reported, so boot and reset start from a clean IDLE with no special-casing.
  private static final double[] lastReportSec = new double[Action.values().length];

  private static DoubleSupplier clockForTest;

  // The dashboard's own readiness evaluator (same rule as the commands, own debounce state) and
  // its latest result. Before the first update() the placeholder matches a freshly booted robot:
  // enabled-or-not, the flywheel is not yet at shot speed.
  private static final ShotReadiness shotReadiness = new ShotReadiness();
  private static ShotReadiness.Evaluation latestEvaluation = notReadyPlaceholder();

  static {
    resetForTest();
  }

  private OperatorStatus() {}

  /**
   * Heartbeat: call every loop while the action holds; a report expires after {@value
   * #REPORT_EXPIRY_SEC} seconds.
   */
  public static void report(Action action) {
    lastReportSec[action.ordinal()] = nowSec();
  }

  /** Effective action: among fresh reports the highest priority wins; nothing fresh is IDLE. */
  public static Action currentAction() {
    double now = nowSec();
    for (Action action : PRIORITY) {
      if (now - lastReportSec[action.ordinal()] <= REPORT_EXPIRY_SEC) {
        return action;
      }
    }
    return Action.IDLE;
  }

  /**
   * Refreshes the shared readiness evaluation against the dashboard shot configuration (full shot
   * speed, plus the Hood Angle (%) entry). Reads subsystem inputs and {@link DashboardControls}
   * only and touches no NetworkTables entries, so it is safe in every mode including replay; call
   * every robotPeriodic loop.
   */
  public static void update() {
    latestEvaluation =
        shotReadiness.update(
            DrumConstants.MaxShotVelocity, DashboardControls.getHoodTargetDeg() / 360.0);
  }

  /** True while feeding is allowed: robot enabled, no jam clearing in progress, shot ready. */
  public static boolean feedAllowed() {
    return !DriverStation.isDisabled()
        && currentAction() != Action.CLEARING_JAM
        && latestEvaluation.rawReady();
  }

  /** Why feeding is blocked right now, or "" when it is allowed. */
  public static String feedBlockedReason() {
    if (DriverStation.isDisabled()) {
      return ShotReadiness.REASON_DISABLED;
    }
    if (currentAction() == Action.CLEARING_JAM) {
      return ShotReadiness.REASON_CLEARING_JAM;
    }
    return latestEvaluation.blockReason();
  }

  /**
   * Writes the operator panel entries to Elastic; call at 10 Hz. The key names are an external
   * contract consumed by elastic-layout.json. No-op in replay.
   */
  public static void publish(int controllerPort) {
    if (Logger.hasReplaySource()) {
      return;
    }
    SmartDashboard.putString("Robot Action", displayName(currentAction()));
    SmartDashboard.putBoolean("Feed Allowed", feedAllowed());
    SmartDashboard.putString("Feed Blocked Reason", feedBlockedReason());
    SmartDashboard.putNumber(
        "Drum Target Speed (rot/s)", Drum.mInstance.getVelocitySetpointRotPerSec());
    SmartDashboard.putNumber("Drum Actual Speed (rot/s)", Drum.mInstance.getVelocityRotPerSec());
    SmartDashboard.putNumber("Battery Voltage (V)", RobotController.getBatteryVoltage());
    SmartDashboard.putBoolean(
        "Controller " + controllerPort + " Connected",
        DriverStation.isJoystickConnected(controllerPort));
  }

  /**
   * Clears every remembered heartbeat and restarts the readiness debounce; restore an injected
   * clock separately (null argument).
   */
  public static void resetForTest() {
    Arrays.fill(lastReportSec, Double.NEGATIVE_INFINITY);
    shotReadiness.reset();
    latestEvaluation = notReadyPlaceholder();
  }

  /** Test hook: replace the clock (seconds). Pass null to restore Logger-based time. */
  public static void setClockForTest(DoubleSupplier secondsSupplierOrNull) {
    clockForTest = secondsSupplierOrNull;
  }

  private static ShotReadiness.Evaluation notReadyPlaceholder() {
    return new ShotReadiness.Evaluation(
        false, false, false, false, false, ShotReadiness.REASON_DRUM_NOT_AT_SPEED);
  }

  private static String displayName(Action action) {
    return switch (action) {
      case IDLE -> "Idle";
      case INTAKING -> "Intaking";
      case SPINNING_UP -> "Spinning Up";
      case FEEDING -> "Feeding";
      case WAITING_RECOVERY -> "Waiting for Flywheel/Hood";
      case CLEARING_JAM -> "Clearing Jam";
      case REVERSING_LOCAL -> "Reversing (Feeder/Indexer)";
      case REVERSING_FULL -> "Full Reverse (Eject)";
    };
  }

  private static double nowSec() {
    return clockForTest != null ? clockForTest.getAsDouble() : Logger.getTimestamp() / 1e6;
  }
}
