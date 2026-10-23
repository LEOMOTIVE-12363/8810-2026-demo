package frc.robot.util;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import frc.robot.commands.ShootingCommand.ShootingConstants;
import frc.robot.subsystems.Drum.Drum;
import frc.robot.subsystems.Hood.Hood;

/**
 * The single source of truth for "may we feed fuel into the shooter right now". The Shooting
 * command and the Manual command each own an instance and report their own control decision to the
 * operator dashboard ({@link OperatorStatus}); the dashboard's own instance is only a fallback
 * preview for when no command is running, so the panel can never drift from the control decision.
 *
 * <p>Pause decisions use {@link Evaluation#rawReady()}: the moment the flywheel or hood leaves
 * tolerance, feeding stops. Resume decisions use {@link Evaluation#stableReady()}: readiness must
 * hold for the full {@value ShootingConstants#READY_DEBOUNCE_SEC} s debounce, so the feed path does
 * not chatter on and off at the tolerance boundary. All tolerances and timings are the existing
 * {@link ShootingConstants} values — no new real-robot parameters were invented here.
 *
 * <p>An instance reads subsystem inputs and {@link DashboardControls} only and touches no
 * NetworkTables entries, so it is safe in every mode including replay. Call {@link #reset()} from
 * {@code Command.initialize()} for a fresh debounce baseline.
 */
public final class ShotReadiness {
  public static final String REASON_DISABLED = "Robot disabled";
  public static final String REASON_FLYWHEEL_ZERO = "Flywheel limit is 0%";
  public static final String REASON_DRUM_NOT_AT_SPEED = "Flywheel not at speed";
  public static final String REASON_HOOD_NOT_AT_ANGLE = "Hood not at angle";
  public static final String REASON_CLEARING_JAM = "Clearing indexer jam";
  public static final String REASON_STABILIZING = "Stabilizing shot";
  public static final String REASON_NONE = "";

  /**
   * One evaluation of the readiness rule. {@code rawReady} is the instantaneous answer, {@code
   * stableReady} the debounced one; {@code blockReason} is the first unmet condition in priority
   * order, or {@link #REASON_NONE} when raw-ready. The disabled and clearing-jam reasons are
   * layered on by callers, not produced here.
   */
  public record Evaluation(
      boolean flywheelZero,
      boolean drumReady,
      boolean hoodReady,
      boolean rawReady,
      boolean stableReady,
      String blockReason) {}

  // Requires raw readiness to hold continuously before stableReady turns true; recreated by
  // reset() because a stale debouncer that last saw "ready" would re-pass instantly on the first
  // sample instead of requiring the full debounce time again. Note the 2026 Debouncer anchors its
  // timer at construction and only re-anchors when calculate() sees the baseline (false) value —
  // so a state that deliberately skips evaluation (UNJAM) must call holdNotReady() every loop, or
  // the first true sample after a quiet gap longer than the debounce window would pass instantly.
  private Debouncer readyDebouncer;

  public ShotReadiness() {
    reset();
  }

  /** Fresh debounce baseline; call from {@code Command.initialize()}. */
  public void reset() {
    readyDebouncer = new Debouncer(ShootingConstants.READY_DEBOUNCE_SEC, DebounceType.kRising);
  }

  /**
   * Feeds the debouncer a not-ready sample without evaluating the rule. States that skip {@link
   * #update} on purpose (UNJAM never lets the gate suppress a back-off) must call this every loop
   * to keep the debounce timer anchored, so the post-skip recovery still requires the full stable
   * window.
   */
  public void holdNotReady() {
    readyDebouncer.calculate(false);
  }

  /**
   * Evaluates the rule against the given shot configuration: drum target in motor rot/s (the
   * Flywheel Speed Limit (%) is applied inside {@link Drum#isAtVelocity}), hood target in mechanism
   * rotations.
   */
  public Evaluation update(double drumTargetRotps, double hoodTargetRot) {
    boolean flywheelZero = DashboardControls.getFlywheelScale() <= 0.0;
    boolean drumReady =
        Drum.mInstance.isAtVelocity(
            drumTargetRotps, ShootingConstants.SHOOTER_VELOCITY_TOLERANCE_ROTPS);
    boolean hoodReady =
        Math.abs(Hood.mInstance.getPositionRot() - hoodTargetRot)
            < ShootingConstants.HOOD_ANGLE_TOLERANCE_ROT;
    boolean rawReady = drumReady && hoodReady;
    boolean stableReady = readyDebouncer.calculate(rawReady);
    String blockReason =
        flywheelZero
            ? REASON_FLYWHEEL_ZERO
            : !drumReady
                ? REASON_DRUM_NOT_AT_SPEED
                : !hoodReady ? REASON_HOOD_NOT_AT_ANGLE : REASON_NONE;
    return new Evaluation(flywheelZero, drumReady, hoodReady, rawReady, stableReady, blockReason);
  }

  /**
   * Block reason for a command that is still waiting to (re)start feeding: the raw reason when a
   * condition is unmet, {@link #REASON_STABILIZING} while the debounce window runs on an
   * already-raw-ready shot, and {@link #REASON_NONE} once stable-ready.
   */
  public static String waitingBlockReason(Evaluation evaluation) {
    if (evaluation.stableReady()) {
      return REASON_NONE;
    }
    return evaluation.rawReady() ? REASON_STABILIZING : evaluation.blockReason();
  }
}
