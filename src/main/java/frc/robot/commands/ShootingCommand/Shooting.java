package frc.robot.commands.ShootingCommand;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drum.Drum;
import frc.robot.subsystems.Drum.DrumConstants;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.IntakeDeploy.IntakeDeploy;
import frc.robot.subsystems.IntakeDeploy.IntakeDeployConstants;
import frc.robot.subsystems.IntakeRoller.IntakeRoller;
import frc.robot.util.BallFlowTracker;
import frc.robot.util.DashboardControls;
import frc.robot.util.OperatorStatus;
import frc.robot.util.ShotReadiness;
import org.littletonrobotics.junction.Logger;

/**
 * Demo shooting: spins the drum and sets the hood from the Elastic Flywheel Speed Limit (%) and
 * Hood Angle (%) entries, then feeds once both are on target. No aiming, vision, or pose/distance
 * input — the drivetrain is not required, so the driver keeps normal control while shooting.
 *
 * <p>Readiness is gated continuously through the whole volley, not just during SPIN_UP, via the
 * shared {@link ShotReadiness} rule (the same rule the operator dashboard shows): flywheel at speed
 * AND hood at angle, with a 0% Flywheel Speed Limit always blocking feeding (Drum.isAtVelocity is
 * false at 0%). Losing readiness mid-volley pauses feeding on that very loop — the feed-path motors
 * stop immediately while the drum and hood keep being driven to target — and regaining stable
 * readiness auto-resumes the volley exactly where it left off, without the driver releasing the
 * button. A pause freezes the retract timer mid-count, so paused time never counts toward the
 * retract delay.
 *
 * <p>UNJAM always runs its back-off to completion regardless of readiness — clearing a jam is never
 * suppressed by the not-ready gate — then routes through PAUSED instead of resuming directly, so
 * forward feeding restarts only once the shot is stable-ready again (the drum may have sagged while
 * stalled). The jam confirmation window only accumulates while feeding: every non-feeding state
 * (SPIN_UP, PAUSED, UNJAM) keeps feeding the debouncer clear samples, so time spent spinning up,
 * paused, or backing out a jam never pre-fills the window, and the first high-current sample after
 * feeding resumes must still persist for the full debounce before UNJAM trips. The retract timer
 * freezes the moment the back-off starts and the readiness debounce resets at the same time, so
 * unjam time never counts toward the retract delay and the post-unjam shot must re-earn a full
 * stable-ready window — previously a >0.2 s unjam during SHOOT meant an instant RETRACT on resume,
 * and the first ready sample after an unjam meant an instant resume; both behavior changes are
 * deliberate.
 *
 * <p>Releasing the key still ends the command outright: nothing auto-resumes after end(), and the
 * whileTrue binding re-initializes the command (and the readiness debounce) on the next press.
 */
public class Shooting extends Command {
  private enum States {
    SPIN_UP,
    SHOOT,
    RETRACT,
    PAUSED,
    UNJAM
  }

  private States state;

  // Times the delay between starting to feed and retracting the intake. PAUSED stops the timer
  // mid-count and resumes it on return, so pause/unjam time never counts toward the delay.
  private final Timer retractTimer = new Timer();
  private boolean retractTimerStarted;
  private final boolean isEnergySave;

  // Indexer jam detection: stator current must stay above the threshold for the full debounce
  // time before UNJAM triggers, and only feeding loops count — every non-feeding state feeds the
  // debouncer a clear sample each loop (see holdJamNotDetected()), so spin-up, pause, and back-off
  // time never pre-fills the confirmation window.
  private Debouncer jamDebouncer;
  // Times the fixed backwards-run of the indexer in UNJAM.
  private final Timer unjamTimer = new Timer();
  // The feeding state (SHOOT or RETRACT) to resume once the unjam back-off finishes.
  private States stateAfterUnjam = States.SHOOT;
  // The feeding state (SHOOT or RETRACT) PAUSED returns to once the shot is stable-ready again.
  private States stateAfterPause;

  // Shared readiness rule (identical to Manual; the operator panel follows this command's reported
  // decision), with this command's own debounce state; lastEvaluation backs the Shooting/*Ready
  // log keys and the panel decision, including during UNJAM where deliberately no fresh evaluation
  // runs.
  private final ShotReadiness readiness = new ShotReadiness();
  private ShotReadiness.Evaluation lastEvaluation;

  public Shooting(boolean isEnergySave) {
    addRequirements(
        Drum.mInstance,
        Feeder.mInstance,
        Hood.mInstance,
        Indexer.mInstance,
        IntakeDeploy.mInstance,
        IntakeRoller.mInstance);
    this.isEnergySave = isEnergySave;
  }

  @Override
  public void initialize() {
    state = States.SPIN_UP;
    retractTimerStarted = false;
    retractTimer.stop();
    retractTimer.reset();
    readiness.reset();
    resetJamDetection();
  }

  /** Hood target in mechanism rotations, from the Hood Angle (%) entry. */
  private double hoodTargetRot() {
    return DashboardControls.getHoodTargetDeg() / 360.;
  }

  /**
   * Drive the flywheel and hood to the dashboard setpoints. The drum is asked for full shot speed;
   * Drum applies Flywheel Speed Limit (%) to it, so the percentage is applied exactly once.
   */
  private void prepareShooter() {
    Drum.mInstance.setVelocityRotPerSec(DrumConstants.MaxShotVelocity);
    Hood.mInstance.setPositionRot(hoodTargetRot());
    Logger.recordOutput(
        "Shooting/drumSetpointRotps",
        DashboardControls.applyFlywheelLimit(DrumConstants.MaxShotVelocity));
    Logger.recordOutput("Shooting/hoodSetpointDeg", DashboardControls.getHoodTargetDeg());
  }

  /**
   * One readiness evaluation for this loop against the dashboard shot configuration, logging the
   * readiness keys the DriverDashboard Elastic tab consumes. Every gating state calls this exactly
   * once per loop; UNJAM deliberately does not — a jam back-off must never be suppressed by the
   * not-ready gate.
   */
  private ShotReadiness.Evaluation updateReadiness() {
    lastEvaluation = readiness.update(DrumConstants.MaxShotVelocity, hoodTargetRot());
    Logger.recordOutput("Shooting/drumReady", lastEvaluation.drumReady());
    Logger.recordOutput("Shooting/hoodReady", lastEvaluation.hoodReady());
    return lastEvaluation;
  }

  /**
   * Start jam detection from a clean baseline. Only initialize() needs a fresh debouncer; the
   * non-feeding states keep the running one re-anchored via holdJamNotDetected().
   */
  private void resetJamDetection() {
    jamDebouncer = new Debouncer(ShootingConstants.INDEXER_JAM_DEBOUNCE_SEC, DebounceType.kRising);
  }

  /**
   * Feed the jam debouncer a clear sample without reading the mechanism. A kRising debouncer
   * re-anchors its timer only on baseline (clear) samples and lets it free-run otherwise, so every
   * non-feeding state must call this each loop: otherwise time spent spinning up, paused, or
   * backing out a jam would silently count toward the confirmation window, and the first
   * high-current sample after feeding resumed would trip UNJAM instantly instead of having to
   * persist for the full debounce.
   */
  private void holdJamNotDetected() {
    jamDebouncer.calculate(false);
  }

  /** True when the indexer stator current has been above the jam threshold for the debounce. */
  private boolean isIndexerJammed() {
    boolean jammed =
        jamDebouncer.calculate(
            Indexer.mInstance.getStatorCurrentAmps() > ShootingConstants.INDEXER_JAM_CURRENT_AMPS);
    Logger.recordOutput("Shooting/indexerJammed", jammed);
    return jammed;
  }

  /**
   * SHOOT/RETRACT -> UNJAM when the indexer stalls. Returns true when the transition fired;
   * remembers the interrupted state so the volley resumes exactly where it left off (through
   * PAUSED, see unjam()). The retract timer freezes the moment the back-off starts and the
   * readiness debounce resets, so after the back-off the shot must re-earn a fresh stable-ready
   * window before feeding resumes.
   */
  private boolean checkJamAndStartUnjam(States resumeState) {
    if (!isIndexerJammed()) {
      return false;
    }
    stateAfterUnjam = resumeState;
    if (retractTimerStarted) {
      retractTimer.stop();
    }
    readiness.reset();
    lastEvaluation = null;
    state = States.UNJAM;
    unjamTimer.restart();
    return true;
  }

  /**
   * Back the stalled fuel out of the indexer, then re-gate through PAUSED: forward feeding resumes
   * only once the shot is stable-ready again. The back-off runs to completion regardless of
   * readiness — clearing a jam must never be suppressed by the not-ready gate. Jam detection is
   * held clear throughout the back-off, so the post-unjam volley starts a fresh confirmation
   * window.
   */
  private void unjam() {
    // Keep holding the flywheel/hood so shooting resumes immediately after the back-off.
    prepareShooter();
    IntakeRoller.mInstance.setV(0);
    Feeder.mInstance.setV(-ShootingConstants.UNJAM_Feeder_Current.getAsDouble());
    Indexer.mInstance.setVelocityRotPerSec(-ShootingConstants.UNJAM_INDEXER_Current.getAsDouble());
    IntakeDeploy.mInstance.setPositionCentimeter(55);
    // The rule is not evaluated during a back-off, but the debouncer must keep seeing not-ready
    // samples — its timer only re-anchors on a baseline sample, so a quiet 0.5 s back-off would
    // otherwise let the first post-unjam ready sample pass the stable window instantly.
    readiness.holdNotReady();
    // Same for the jam debouncer: the back-off must not pre-fill the next confirmation window.
    holdJamNotDetected();

    if (unjamTimer.hasElapsed(ShootingConstants.UNJAM_DURATION_SEC.getAsDouble())) {
      enterPause(stateAfterUnjam);
    }
  }

  private void spinUp() {
    prepareShooter();
    // Keep the feed path stopped until the flywheel and hood are on target.
    IntakeRoller.mInstance.setV(0);
    Indexer.mInstance.setV(0);
    Feeder.mInstance.setV(0);
    // Spin-up is not feeding: keep the jam debouncer anchored, or a long real spin-up would
    // pre-fill the confirmation window and the first high-current sample would trip UNJAM.
    holdJamNotDetected();

    if (updateReadiness().stableReady()) {
      state = States.SHOOT;
    }
  }

  /** Drive the full feed path (intake roller, indexer, feeder) at the tuned shooting speeds. */
  private void runFeed() {
    if (IntakeDeploy.mInstance.getPositionCentimeter() <= 30) {
      IntakeRoller.mInstance.setVelocityRotPerSec(0);
    } else {
      IntakeRoller.mInstance.setVelocityRotPerSec(
          ShootingConstants.IntakeRollerRotpsTunable.getAsDouble());
    }
    if (!isEnergySave) {
      Indexer.mInstance.setVelocityRotPerSec(ShootingConstants.IndexerRotpsTunable.getAsDouble());
      Feeder.mInstance.setVelocityRotPerSec(ShootingConstants.FeederRotpsTunable.getAsDouble());
    } else {
      Indexer.mInstance.setCurrent(50);
      Feeder.mInstance.setCurrent(50);
    }
  }

  private void shoot() {
    if (checkJamAndStartUnjam(States.SHOOT)) {
      return;
    }
    // Losing readiness mid-volley pauses feeding on this very loop — the feed path stops now, not
    // next loop; PAUSED auto-resumes the volley once the shot is stable-ready again.
    if (!updateReadiness().rawReady()) {
      enterPause(States.SHOOT);
      holdPauseOutputs();
      return;
    }

    prepareShooter();
    runFeed();

    if (!retractTimerStarted) {
      retractTimer.restart();
      retractTimerStarted = true;
    }
    if (retractTimer.hasElapsed(ShootingConstants.RETRACT_DELAY_SEC.getAsDouble())) {
      state = States.RETRACT;
    }
  }

  private void retract() {
    if (checkJamAndStartUnjam(States.RETRACT)) {
      return;
    }
    if (!updateReadiness().rawReady()) {
      enterPause(States.RETRACT);
      holdPauseOutputs();
      return;
    }

    // Keep shooting while pulling the intake back.
    prepareShooter();
    runFeed();
    IntakeDeploy.mInstance.setPositionCentimeter(
        ShootingConstants.INTAKE_RETRACT_POSITION_CM.getAsDouble(), 80, 1000, 0);
  }

  /**
   * Freeze the volley exactly where it is: remember the feeding state to resume and freeze the
   * retract timer mid-count (Timer.start() on resume picks the accumulated count back up, so
   * feeding time during a pause never counts toward the retract delay). Jam detection is not reset
   * here — paused() keeps feeding the debouncer clear samples, which re-anchors it every loop.
   */
  private void enterPause(States resume) {
    stateAfterPause = resume;
    if (retractTimerStarted) {
      retractTimer.stop();
    }
    state = States.PAUSED;
  }

  /**
   * The outputs every paused loop holds: drum and hood keep driving to target so the volley can
   * resume the moment readiness returns, only the feed path stops; a paused RETRACT keeps pulling
   * the intake back (the intake never re-extends mid-volley). Runs both from paused() and on the
   * exact loop a pause is entered, so losing readiness stops the feed path that same loop.
   */
  private void holdPauseOutputs() {
    prepareShooter();
    IntakeRoller.mInstance.setV(0);
    Indexer.mInstance.setV(0);
    Feeder.mInstance.setV(0);
    if (stateAfterPause == States.RETRACT) {
      IntakeDeploy.mInstance.setPositionCentimeter(
          ShootingConstants.INTAKE_RETRACT_POSITION_CM.getAsDouble(), 80, 1000, 0);
    }
  }

  /**
   * Hold the shooter on target with the feed path stopped until the shot is stable-ready again,
   * then resume the remembered feeding state. A pause never resets the volley — the retract timer
   * resumes mid-count and a paused RETRACT never re-triggers the retract.
   */
  private void paused() {
    holdPauseOutputs();
    // Paused time must not count toward the jam confirmation window: keep feeding the debouncer
    // clear samples so the first high-current reading after the resume still has to persist for
    // the full debounce before UNJAM trips.
    holdJamNotDetected();

    if (updateReadiness().stableReady()) {
      if (retractTimerStarted) {
        retractTimer.start();
      }
      state = stateAfterPause;
    }
  }

  @Override
  public void execute() {
    switch (state) {
      case SPIN_UP -> spinUp();
      case SHOOT -> shoot();
      case RETRACT -> retract();
      case PAUSED -> paused();
      case UNJAM -> unjam();
    }
    // Heartbeat the semantic feed state for the LED ball-flow display; display-only. SPIN_UP and
    // PAUSED feed nothing, so they report nothing and the strip keeps its idle effect; UNJAM
    // reverses only the feeder/indexer, so it reports a local reverse, never a whole-path one.
    switch (state) {
      case SHOOT, RETRACT -> BallFlowTracker.report(BallFlowTracker.FlowKind.FEED);
      case UNJAM -> BallFlowTracker.report(BallFlowTracker.FlowKind.LOCAL_REVERSE);
      case SPIN_UP, PAUSED -> {}
    }
    // The same heartbeat for the operator panel: it follows the mechanism's actual decision,
    // including waiting-for-recovery while paused.
    switch (state) {
      case SPIN_UP -> OperatorStatus.report(OperatorStatus.Action.SPINNING_UP);
      case SHOOT, RETRACT -> OperatorStatus.report(OperatorStatus.Action.FEEDING);
      case PAUSED -> OperatorStatus.report(OperatorStatus.Action.WAITING_RECOVERY);
      case UNJAM -> OperatorStatus.report(OperatorStatus.Action.CLEARING_JAM);
    }
    // And the panel's feed decision comes from this command's own evaluation, so the dashboard
    // can never drift from the control loop. UNJAM reports nothing: the CLEARING_JAM action
    // already overrides the panel to blocked.
    switch (state) {
      case SHOOT, RETRACT -> OperatorStatus.reportFeedDecision(true, ShotReadiness.REASON_NONE);
      case SPIN_UP, PAUSED -> {
        if (lastEvaluation != null) {
          OperatorStatus.reportFeedDecision(
              lastEvaluation.stableReady(), ShotReadiness.waitingBlockReason(lastEvaluation));
        }
      }
      case UNJAM -> {}
    }
    Logger.recordOutput("Shooting/state", state.toString());
    Logger.recordOutput(
        "Shooting/stableReady", lastEvaluation != null && lastEvaluation.stableReady());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    OperatorStatus.clearFeedDecision();
    Drum.mInstance.setVelocityRotPerSec(DrumConstants.StowVelocity);
    Hood.mInstance.setPositionRot(0);
    IntakeRoller.mInstance.stop();
    IntakeDeploy.mInstance.setPositionCentimeter(IntakeDeployConstants.IntakeOutPosition);
    Indexer.mInstance.stop();
    Feeder.mInstance.stop();
  }
}
