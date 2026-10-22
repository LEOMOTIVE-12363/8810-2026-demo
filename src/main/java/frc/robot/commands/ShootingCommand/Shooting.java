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
import org.littletonrobotics.junction.Logger;

/**
 * Demo shooting: spins the drum and sets the hood from the Elastic Flywheel Speed Limit (%) and
 * Hood Angle (%) entries, then feeds once both are on target. No aiming, vision, or pose/distance
 * input — the drivetrain is not required, so the driver keeps normal control while shooting.
 */
public class Shooting extends Command {
  private enum States {
    SPIN_UP,
    SHOOT,
    RETRACT,
    UNJAM
  }

  private States state;

  // Times the delay between starting to feed and retracting the intake.
  private final Timer retractTimer = new Timer();
  private boolean retractTimerStarted;
  private final boolean isEnergySave;

  // Indexer jam detection: stator current must stay above the threshold for the full debounce
  // time before UNJAM triggers (recreated whenever detection restarts, see isIndexerJammed()).
  private Debouncer jamDebouncer;
  // Times the fixed backwards-run of the indexer in UNJAM.
  private final Timer unjamTimer = new Timer();
  // The feeding state (SHOOT or RETRACT) to resume once the unjam back-off finishes.
  private States stateAfterUnjam = States.SHOOT;

  // Requires isReadyToShoot() to hold continuously before SPIN_UP -> SHOOT (recreated in
  // initialize).
  private Debouncer readyDebouncer;

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
    readyDebouncer = new Debouncer(ShootingConstants.READY_DEBOUNCE_SEC, DebounceType.kRising);
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

  /** True once the flywheel and hood are within tolerance of the dashboard setpoints. */
  private boolean isReadyToShoot() {
    boolean drumReady =
        Drum.mInstance.isAtVelocity(
            DrumConstants.MaxShotVelocity, ShootingConstants.SHOOTER_VELOCITY_TOLERANCE_ROTPS);
    boolean hoodReady =
        Math.abs(Hood.mInstance.getPositionRot() - hoodTargetRot())
            < ShootingConstants.HOOD_ANGLE_TOLERANCE_ROT;
    Logger.recordOutput("Shooting/hoodReady", hoodReady);
    Logger.recordOutput("Shooting/drumReady", drumReady);
    return drumReady && hoodReady;
  }

  /**
   * Restart jam detection from a clean baseline. The debouncer must be recreated whenever detection
   * pauses (SPIN_UP, UNJAM): a stale one that last saw "jammed" would re-trip instantly on the
   * first sample instead of requiring the full debounce time again.
   */
  private void resetJamDetection() {
    jamDebouncer = new Debouncer(ShootingConstants.INDEXER_JAM_DEBOUNCE_SEC, DebounceType.kRising);
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
   * remembers the interrupted state so the volley resumes exactly where it left off.
   */
  private boolean checkJamAndStartUnjam(States resumeState) {
    if (!isIndexerJammed()) {
      return false;
    }
    stateAfterUnjam = resumeState;
    state = States.UNJAM;
    unjamTimer.restart();
    return true;
  }

  /** Back the stalled fuel out of the indexer, then resume the interrupted feeding state. */
  private void unjam() {
    // Keep holding the flywheel/hood so shooting resumes immediately after the back-off.
    prepareShooter();
    IntakeRoller.mInstance.setV(0);
    Feeder.mInstance.setV(-ShootingConstants.UNJAM_Feeder_Current.getAsDouble());
    Indexer.mInstance.setVelocityRotPerSec(-ShootingConstants.UNJAM_INDEXER_Current.getAsDouble());
    IntakeDeploy.mInstance.setPositionCentimeter(55);

    if (unjamTimer.hasElapsed(ShootingConstants.UNJAM_DURATION_SEC.getAsDouble())) {
      resetJamDetection();
      state = stateAfterUnjam;
    }
  }

  private void spinUp() {
    prepareShooter();
    // Keep the feed path stopped until the flywheel and hood are on target.
    IntakeRoller.mInstance.setV(0);
    Indexer.mInstance.setV(0);
    Feeder.mInstance.setV(0);

    if (readyDebouncer.calculate(isReadyToShoot())) {
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

    // Keep shooting while pulling the intake back.
    prepareShooter();
    runFeed();
    IntakeDeploy.mInstance.setPositionCentimeter(
        ShootingConstants.INTAKE_RETRACT_POSITION_CM.getAsDouble(), 80, 1000, 0);
  }

  @Override
  public void execute() {
    switch (state) {
      case SPIN_UP -> spinUp();
      case SHOOT -> shoot();
      case RETRACT -> retract();
      case UNJAM -> unjam();
    }
    // Heartbeat the semantic feed state for the LED ball-flow display; display-only. SPIN_UP
    // feeds nothing, so it reports nothing and the strip keeps its idle effect; UNJAM reverses
    // only the feeder/indexer, so it reports a local reverse, never a whole-path one.
    switch (state) {
      case SHOOT, RETRACT -> BallFlowTracker.report(BallFlowTracker.FlowKind.FEED);
      case UNJAM -> BallFlowTracker.report(BallFlowTracker.FlowKind.LOCAL_REVERSE);
      case SPIN_UP -> {}
    }
    Logger.recordOutput("Shooting/state", state.toString());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Drum.mInstance.setVelocityRotPerSec(DrumConstants.StowVelocity);
    Hood.mInstance.setPositionRot(0);
    IntakeRoller.mInstance.stop();
    IntakeDeploy.mInstance.setPositionCentimeter(IntakeDeployConstants.IntakeOutPosition);
    Indexer.mInstance.stop();
    Feeder.mInstance.stop();
  }
}
