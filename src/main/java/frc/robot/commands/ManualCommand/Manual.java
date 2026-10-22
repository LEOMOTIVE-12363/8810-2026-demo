package frc.robot.commands.ManualCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drum.Drum;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.IntakeDeploy.IntakeDeploy;
import frc.robot.subsystems.IntakeRoller.IntakeRoller;
import frc.robot.util.BallFlowTracker;
import frc.robot.util.OperatorStatus;
import frc.robot.util.ShotReadiness;

/**
 * X-button fixed-setpoint shot path: spins the drum and sets the hood to the Manual tunables, then
 * feeds. It shares the Shooting command's readiness rule ({@link ShotReadiness}) so no trigger path
 * can bypass the flywheel-zero / not-at-speed / hood-off-target feed gate: losing readiness
 * mid-SHOOT drops back to PREPARE (the feed path stops while the drum and hood keep being driven to
 * their Manual setpoints), and regaining stable readiness re-enters SHOOT on its own while the
 * button stays held.
 */
public class Manual extends Command {
  private enum States {
    PREPARE,
    SHOOT
  }

  private States state;
  private final ShotReadiness readiness = new ShotReadiness();

  public Manual() {
    addRequirements(
        Drum.mInstance,
        Feeder.mInstance,
        Hood.mInstance,
        Indexer.mInstance,
        IntakeDeploy.mInstance,
        IntakeRoller.mInstance);
  }

  @Override
  public void initialize() {
    state = States.PREPARE;
    readiness.reset();
  }

  /** Evaluates the shared readiness rule once against the Manual drum/hood setpoints. */
  private ShotReadiness.Evaluation updateReadiness() {
    return readiness.update(
        ManualConstants.ShooterRotpsTunable.getAsDouble(),
        ManualConstants.HoodDegTunable.getAsDouble() / 360.);
  }

  /**
   * Drives the drum and hood to the Manual setpoints with the feed path stopped; doubles as the
   * pause state SHOOT falls back to. Transitions to SHOOT only when readiness has held for the full
   * shared debounce — deliberately stricter than the single raw sample it replaced, matching
   * Shooting's SPIN_UP gate.
   */
  private void prepare() {
    Drum.mInstance.setVelocityRotPerSec(ManualConstants.ShooterRotpsTunable.getAsDouble());
    Hood.mInstance.setPositionRot(ManualConstants.HoodDegTunable.getAsDouble() / 360.);
    IntakeRoller.mInstance.setV(0);
    Indexer.mInstance.setV(0);
    Feeder.mInstance.setV(0);
    if (updateReadiness().stableReady()) {
      state = States.SHOOT;
    }
  }

  /**
   * Feeds at the Manual speeds while readiness holds. A single raw-not-ready loop (flywheel limit
   * at 0%, drum off speed, hood off target) drops back to PREPARE before any motor command, so the
   * feed path stops immediately; re-entry requires the full debounce again.
   */
  private void shoot() {
    if (!updateReadiness().rawReady()) {
      state = States.PREPARE;
      return;
    }
    Drum.mInstance.setVelocityRotPerSec(ManualConstants.ShooterRotpsTunable.getAsDouble());
    Hood.mInstance.setPositionRot(ManualConstants.HoodDegTunable.getAsDouble() / 360.);
    IntakeRoller.mInstance.setVelocityRotPerSec(
        ManualConstants.IntakeRollerRotpsTunable.getAsDouble());
    Indexer.mInstance.setVelocityRotPerSec(ManualConstants.IndexerRotpsTunable.getAsDouble());
    Feeder.mInstance.setVelocityRotPerSec(ManualConstants.FeederRotpsTunable.getAsDouble());
  }

  @Override
  public void execute() {
    switch (state) {
      case PREPARE:
        prepare();
        break;
      case SHOOT:
        shoot();
        break;
    }
    // Heartbeat the semantic feed state for the LED ball-flow display; display-only. PREPARE
    // feeds nothing, so it reports nothing and the strip keeps its idle effect.
    if (state == States.SHOOT) {
      BallFlowTracker.report(BallFlowTracker.FlowKind.FEED);
    }
    // Operator panel heartbeat: PREPARE is spinning up, SHOOT is feeding.
    OperatorStatus.report(
        state == States.SHOOT ? OperatorStatus.Action.FEEDING : OperatorStatus.Action.SPINNING_UP);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Drum.mInstance.stop();
    Hood.mInstance.stop();
    IntakeRoller.mInstance.stop();
    Indexer.mInstance.stop();
    Feeder.mInstance.stop();
  }
}
