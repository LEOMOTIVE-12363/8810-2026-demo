package frc.robot.util;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import frc.robot.commands.ShootingCommand.ShootingConstants;
import frc.robot.subsystems.Drum.Drum;
import frc.robot.subsystems.Hood.Hood;

/**
 * The single source of truth for "may we feed fuel into the shooter right now". The Shooting
 * command, the Manual command, and the operator dashboard ({@link OperatorStatus}) each own an
 * instance — identical rules, independent debounce state — so the panel can never drift from the
 * control decision.
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
  // sample instead of requiring the full debounce time again.
  private Debouncer readyDebouncer;

  public ShotReadiness() {
    reset();
  }

  /** Fresh debounce baseline; call from {@code Command.initialize()}. */
  public void reset() {
    readyDebouncer = new Debouncer(ShootingConstants.READY_DEBOUNCE_SEC, DebounceType.kRising);
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
}
