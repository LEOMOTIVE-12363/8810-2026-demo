package frc.robot.commands.ShootingCommand;

import static frc.robot.util.DashboardControlsTestUtil.FLYWHEEL;
import static frc.robot.util.DashboardControlsTestUtil.HOOD;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.lib.io.MotorIO;
import frc.robot.subsystems.Drum.Drum;
import frc.robot.subsystems.Drum.DrumConstants;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Hood.HoodConstants;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.IntakeDeploy.IntakeDeploy;
import frc.robot.subsystems.IntakeRoller.IntakeRoller;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.DashboardControlsTestUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.OperatorStatus;
import frc.robot.util.OperatorStatus.Action;
import frc.robot.util.ShotReadiness;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Runs Shooting against the sim mechanism IO with no Drive, pose, or vision ever created: if the
 * command touched any of them it would throw on the null Drive.mInstance.
 *
 * <p>The instant-setpoint sim snaps Drum/Hood to their setpoints on the next periodic, so a
 * dashboard edit flips readiness for exactly one loop (the evaluation compares the new target
 * against the still-old measured value), and the 0.15 s stable debounce is 8 loops of 0.02 s.
 */
class ShootingTest {
  private static final double EPS = 1e-6;

  @BeforeAll
  static void setUpHal() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
  }

  @AfterEach
  void resetDashboard() {
    Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(null);
    OperatorStatus.resetForTest();
    OperatorStatus.setClockForTest(null);
    DriverStationSim.setEnabled(false);
    DriverStationSim.setDsAttached(false);
    DriverStationSim.notifyNewData();
    DashboardControlsTestUtil.resetDefaults();
  }

  /** One robot loop: run the command, advance time, then update mechanism inputs. */
  private static void step(Shooting shooting) {
    shooting.execute();
    SimHooks.stepTiming(0.02);
    Drum.mInstance.periodic();
    Hood.mInstance.periodic();
    Feeder.mInstance.periodic();
    Indexer.mInstance.periodic();
    IntakeDeploy.mInstance.periodic();
    IntakeRoller.mInstance.periodic();
  }

  /** Same loop, plus the operator-panel periodic the robot would run. */
  private static void stepWithPanel(Shooting shooting) {
    shooting.execute();
    OperatorStatus.update();
    SimHooks.stepTiming(0.02);
    Drum.mInstance.periodic();
    Hood.mInstance.periodic();
    Feeder.mInstance.periodic();
    Indexer.mInstance.periodic();
    IntakeDeploy.mInstance.periodic();
    IntakeRoller.mInstance.periodic();
  }

  /** The same panel loop for the Manual command. */
  private static void stepWithPanel(frc.robot.commands.ManualCommand.Manual manual) {
    manual.execute();
    OperatorStatus.update();
    SimHooks.stepTiming(0.02);
    Drum.mInstance.periodic();
    Hood.mInstance.periodic();
    Feeder.mInstance.periodic();
    Indexer.mInstance.periodic();
    IntakeDeploy.mInstance.periodic();
    IntakeRoller.mInstance.periodic();
  }

  private static double feederSetpoint() {
    return Feeder.mInstance.getVelocitySetpointRotPerSec();
  }

  private static double indexerSetpoint() {
    return Indexer.mInstance.getVelocitySetpointRotPerSec();
  }

  private static double feedSpeed() {
    return ShootingConstants.FeederRotpsTunable.getAsDouble();
  }

  private static boolean isFeeding() {
    return Math.abs(feederSetpoint() - feedSpeed()) < EPS;
  }

  /** Steps until the feeder runs at the tuned feed speed; false when the loop budget ran out. */
  private static boolean stepUntilFeeding(Shooting shooting, int maxLoops) {
    for (int i = 0; i < maxLoops; i++) {
      step(shooting);
      if (isFeeding()) {
        return true;
      }
    }
    return false;
  }

  @Test
  void shootsFromDashboardSettingsWithoutPoseOrVision() {
    assertNull(Drive.mInstance, "test must run with no drivetrain / pose source");
    DashboardControlsTestUtil.set(FLYWHEEL, 50.0);
    DashboardControlsTestUtil.set(HOOD, 100.0);

    Shooting shooting = new Shooting(false);
    assertFalse(shooting.getRequirements().contains(Drive.mInstance));
    shooting.initialize();
    for (int i = 0; i < 20; i++) {
      step(shooting);
    }

    assertEquals(
        0.5 * DrumConstants.MaxShotVelocity, Drum.mInstance.getVelocitySetpointRotPerSec(), EPS);
    assertEquals(HoodConstants.MaxAngleDeg / 360.0, Hood.mInstance.getPositionSetpointRot(), EPS);
    // Reached SHOOT: the feeder is running at the tuned feed speed.
    assertEquals(
        ShootingConstants.FeederRotpsTunable.getAsDouble(),
        Feeder.mInstance.getVelocitySetpointRotPerSec(),
        EPS);

    // Dashboard edits mid-volley still change the commanded setpoints. The readiness gate pauses
    // on the edit loop (the new 5% target reads as not-ready against the still-old measured
    // speed), and entering the pause already drives the drum and hood to the new targets.
    DashboardControlsTestUtil.set(FLYWHEEL, 5.0);
    DashboardControlsTestUtil.set(HOOD, 0.0);
    step(shooting);
    step(shooting);
    assertEquals(
        0.05 * DrumConstants.MaxShotVelocity, Drum.mInstance.getVelocitySetpointRotPerSec(), EPS);
    assertEquals(HoodConstants.MinAngleDeg / 360.0, Hood.mInstance.getPositionSetpointRot(), EPS);

    shooting.end(true);
  }

  @Test
  void zeroFlywheelNeverFeeds() {
    DashboardControlsTestUtil.set(FLYWHEEL, 0.0);

    Shooting shooting = new Shooting(false);
    shooting.initialize();
    for (int i = 0; i < 50; i++) {
      step(shooting);
    }

    assertEquals(0.0, Drum.mInstance.getVelocitySetpointRotPerSec(), EPS);
    assertEquals(0.0, Feeder.mInstance.getVelocitySetpointRotPerSec(), EPS);
    shooting.end(true);
  }

  @Test
  void waitsForStableReadyBeforeFeeding() {
    // Defaults are ready from boot (flywheel 100%, hood target equals the settled position), but
    // feeding still waits for the full 0.15 s stable debounce.
    Shooting shooting = new Shooting(false);
    shooting.initialize();
    for (int i = 0; i < 3; i++) {
      step(shooting);
      assertEquals(0.0, feederSetpoint(), EPS, "still spinning up at loop " + i);
    }
    for (int i = 3; i < 12; i++) {
      step(shooting);
    }
    assertEquals(feedSpeed(), feederSetpoint(), EPS, "feeding once stable-ready");
    shooting.end(true);
  }

  @Test
  void pausesOnZeroFlywheelMidVolleyAndAutoResumes() {
    Shooting shooting = new Shooting(false);
    shooting.initialize();
    assertTrue(stepUntilFeeding(shooting, 30), "reached SHOOT");

    DashboardControlsTestUtil.set(FLYWHEEL, 0.0);
    step(shooting);
    assertEquals(0.0, feederSetpoint(), EPS, "feed path stopped on the detection loop");
    assertEquals(
        0.0,
        Drum.mInstance.getVelocitySetpointRotPerSec(),
        EPS,
        "drum still commanded, held at zero by the limit");
    for (int i = 0; i < 18; i++) {
      step(shooting);
      assertEquals(0.0, feederSetpoint(), EPS, "still paused at hold loop " + i);
    }

    DashboardControlsTestUtil.set(FLYWHEEL, 100.0);
    assertTrue(
        stepUntilFeeding(shooting, 15),
        "auto-resumed within the debounce window, no end/initialize in between");
    assertEquals(feedSpeed(), feederSetpoint(), EPS);
    shooting.end(true);
  }

  @Test
  void pauseFreezesRetractTimer() {
    Shooting shooting = new Shooting(false);
    shooting.initialize();
    assertTrue(stepUntilFeeding(shooting, 30), "reached SHOOT");

    // Pause almost immediately after feeding starts, long before the 0.2 s retract delay
    // accumulates, and hold the pause for twice that delay: without the freeze, RETRACT would
    // fire on the very first loop after resume.
    DashboardControlsTestUtil.set(FLYWHEEL, 0.0);
    for (int i = 0; i < 20; i++) {
      step(shooting);
    }
    DashboardControlsTestUtil.set(FLYWHEEL, 100.0);
    assertTrue(stepUntilFeeding(shooting, 15), "resumed feeding");

    double retractCm = ShootingConstants.INTAKE_RETRACT_POSITION_CM.getAsDouble();
    for (int i = 0; i < 3; i++) {
      assertTrue(
          Math.abs(IntakeDeploy.mInstance.getPositionSetpointRot() - retractCm) > EPS,
          "retract timer resumed mid-count, no instant RETRACT at loop " + i);
      step(shooting);
    }
    boolean retracted = false;
    for (int i = 0; i < 15 && !retracted; i++) {
      step(shooting);
      retracted = Math.abs(IntakeDeploy.mInstance.getPositionSetpointRot() - retractCm) < EPS;
    }
    assertTrue(retracted, "RETRACT fired after the remaining feed time");
    shooting.end(true);
  }

  @Test
  void interruptDuringPauseDoesNotAutoResume() {
    Shooting shooting = new Shooting(false);
    shooting.initialize();
    assertTrue(stepUntilFeeding(shooting, 30), "reached SHOOT");

    DashboardControlsTestUtil.set(FLYWHEEL, 0.0);
    step(shooting);
    assertEquals(0.0, feederSetpoint(), EPS, "paused on the detection loop");

    shooting.end(true);
    assertEquals(0.0, feederSetpoint(), EPS, "end() stopped the feeder");

    // Releasing and re-pressing the button re-initializes the same instance; with the limit
    // still at 0 the fresh volley starts from SPIN_UP and never feeds.
    shooting.initialize();
    for (int i = 0; i < 30; i++) {
      step(shooting);
      assertEquals(0.0, feederSetpoint(), EPS, "no auto-resume of the old volley at loop " + i);
    }
    shooting.end(true);
  }

  @Test
  void midVolleyTargetChangePausesBriefly() {
    Shooting shooting = new Shooting(false);
    shooting.initialize();
    assertTrue(stepUntilFeeding(shooting, 30), "reached SHOOT at 100%");

    // New drum target 33.5 rot/s vs the still-measured 67: not ready until the sim snaps.
    DashboardControlsTestUtil.set(FLYWHEEL, 50.0);
    step(shooting);
    assertEquals(0.0, feederSetpoint(), EPS, "feeding paused on the detection loop");

    assertTrue(stepUntilFeeding(shooting, 15), "auto-resumed at the new target");
    assertEquals(
        0.5 * DrumConstants.MaxShotVelocity,
        Drum.mInstance.getVelocitySetpointRotPerSec(),
        EPS,
        "drum now commanded at 50%");
    shooting.end(true);
  }

  @Test
  void unjamRoutesThroughReadinessGateBeforeResuming() {
    Shooting shooting = new Shooting(false);
    shooting.initialize();
    assertTrue(stepUntilFeeding(shooting, 30), "reached SHOOT");

    Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(100.0);
    boolean unjamming = false;
    for (int i = 0; i < 20 && !unjamming; i++) {
      step(shooting);
      unjamming = indexerSetpoint() < -EPS;
    }
    assertTrue(unjamming, "UNJAM active: indexer backing off");
    Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(null);

    // Run out the 0.5 s back-off; the indexer must keep reversing until the timer elapses.
    boolean unjamOver = false;
    for (int i = 0; i < 40 && !unjamOver; i++) {
      step(shooting);
      unjamOver = Math.abs(indexerSetpoint()) < EPS;
      if (!unjamOver) {
        assertTrue(indexerSetpoint() < -EPS, "indexer keeps reversing until the back-off ends");
      }
    }
    assertTrue(unjamOver, "back-off ran to completion");
    // The first loop without the reverse is PAUSED: the readiness gate runs before any forward
    // feed, and the readiness debounce was reset at UNJAM entry, so the shot must re-earn a full
    // 0.15 s stable window even though the sim drum never sagged during the stall. (On the real
    // robot the drum dip would hold PAUSED even longer.)
    assertEquals(0.0, feederSetpoint(), EPS, "not feeding on the first loop after unjam");

    boolean resumed = false;
    for (int i = 0; i < 15 && !resumed; i++) {
      step(shooting);
      if (isFeeding()) {
        resumed = true;
      } else {
        assertTrue(indexerSetpoint() < EPS, "indexer never runs forward while paused");
      }
    }
    assertTrue(resumed, "feeding resumed once stable-ready");
    shooting.end(true);
  }

  @Test
  void unjamIsNotSuppressedByNotReady() {
    Shooting shooting = new Shooting(false);
    shooting.initialize();
    assertTrue(stepUntilFeeding(shooting, 30), "reached SHOOT");

    Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(100.0);
    boolean unjamming = false;
    for (int i = 0; i < 20 && !unjamming; i++) {
      step(shooting);
      unjamming = indexerSetpoint() < -EPS;
    }
    assertTrue(unjamming, "UNJAM active: indexer backing off");
    Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(null);

    // Losing readiness mid-unjam must not kill the back-off.
    DashboardControlsTestUtil.set(FLYWHEEL, 0.0);
    for (int i = 0; i < 10; i++) {
      step(shooting);
      assertTrue(indexerSetpoint() < -EPS, "back-off continues despite 0% flywheel, loop " + i);
    }
    boolean unjamOver = false;
    for (int i = 0; i < 20 && !unjamOver; i++) {
      step(shooting);
      unjamOver = Math.abs(indexerSetpoint()) < EPS;
    }
    assertTrue(unjamOver, "back-off ran to completion despite the not-ready gate");

    for (int i = 0; i < 30; i++) {
      step(shooting);
      assertEquals(0.0, feederSetpoint(), EPS, "paused while the flywheel limit is 0, loop " + i);
    }

    DashboardControlsTestUtil.set(FLYWHEEL, 100.0);
    assertTrue(stepUntilFeeding(shooting, 15), "feeding resumed once stable-ready");
    shooting.end(true);
  }

  @Test
  void operatorPanelMatchesFeedingDecision() {
    OperatorStatus.resetForTest();
    OperatorStatus.setClockForTest(Timer::getFPGATimestamp);
    // DriverStation.isEnabled() is enabled && dsAttached; notifyNewData() refreshes the cache.
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    Shooting shooting = new Shooting(false);
    shooting.initialize();
    boolean feeding = false;
    for (int i = 0; i < 30 && !feeding; i++) {
      stepWithPanel(shooting);
      feeding = isFeeding();
    }
    assertTrue(feeding, "reached SHOOT");
    assertEquals(Action.FEEDING, OperatorStatus.currentAction());
    assertTrue(OperatorStatus.feedAllowed(), "ready + enabled + no jam");

    DashboardControlsTestUtil.set(FLYWHEEL, 0.0);
    stepWithPanel(shooting);
    assertEquals(Action.WAITING_RECOVERY, OperatorStatus.currentAction());
    assertFalse(OperatorStatus.feedAllowed());
    assertEquals(ShotReadiness.REASON_FLYWHEEL_ZERO, OperatorStatus.feedBlockedReason());

    // Key released: the command stops heartbeating; after the 0.15 s expiry the panel is Idle.
    shooting.end(true);
    for (int i = 0; i < 10; i++) {
      SimHooks.stepTiming(0.02);
      Drum.mInstance.periodic();
      Hood.mInstance.periodic();
    }
    OperatorStatus.update();
    assertEquals(Action.IDLE, OperatorStatus.currentAction());
  }

  private static String stateOf(Shooting s) throws Exception {
    var f = Shooting.class.getDeclaredField("state");
    f.setAccessible(true);
    return f.get(s).toString();
  }

  private static double retractElapsed(Shooting s) throws Exception {
    var f = Shooting.class.getDeclaredField("retractTimer");
    f.setAccessible(true);
    return ((Timer) f.get(s)).get();
  }

  /** Overrides a tunable as an Elastic edit would, refreshing the cached value by hand. */
  private static void overrideTunable(LoggedTunableNumber tunable, String key, double value)
      throws Exception {
    NetworkTableInstance.getDefault().getEntry("/Tuning/" + key).setDouble(value);
    var f = LoggedTunableNumber.class.getDeclaredField("dashboardNumber");
    f.setAccessible(true);
    ((LoggedNetworkNumber) f.get(tunable)).periodic();
  }

  @Test
  void acceptanceStopsOnFirstNotReadyLoop() {
    Shooting s = new Shooting(false);
    s.initialize();
    assertTrue(stepUntilFeeding(s, 30));
    DashboardControlsTestUtil.set(FLYWHEEL, 0.0);
    step(s);
    assertEquals(0.0, feederSetpoint(), EPS, "must stop on detection loop");
    s.end(true);
  }

  @Test
  void acceptanceUnjamFreezesRetractTimer() throws Exception {
    Shooting s = new Shooting(false);
    s.initialize();
    assertTrue(stepUntilFeeding(s, 30));
    Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(100.0);
    for (int i = 0; i < 30 && !stateOf(s).equals("UNJAM"); i++) step(s);
    assertEquals("UNJAM", stateOf(s));
    double before = retractElapsed(s);
    for (int i = 0; i < 5; i++) step(s);
    assertEquals(before, retractElapsed(s), EPS, "UNJAM time must not count");
    s.end(true);
  }

  @Test
  void acceptanceUnjamRequiresFreshStableWindow() throws Exception {
    Shooting s = new Shooting(false);
    s.initialize();
    assertTrue(stepUntilFeeding(s, 30));
    Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(100.0);
    for (int i = 0; i < 30 && !stateOf(s).equals("UNJAM"); i++) step(s);
    assertEquals("UNJAM", stateOf(s));
    Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(null);
    for (int i = 0; i < 40 && stateOf(s).equals("UNJAM"); i++) step(s);
    assertEquals("PAUSED", stateOf(s));
    step(s);
    assertEquals("PAUSED", stateOf(s), "one sample cannot prove stable recovery after unjam");
    s.end(true);
  }

  @Test
  void acceptancePanelUsesManualTarget() {
    OperatorStatus.resetForTest();
    OperatorStatus.setClockForTest(Timer::getFPGATimestamp);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    var m = new frc.robot.commands.ManualCommand.Manual();
    m.initialize();
    for (int i = 0; i < 30; i++) {
      m.execute();
      OperatorStatus.update();
      SimHooks.stepTiming(.02);
      Drum.mInstance.periodic();
      Hood.mInstance.periodic();
      Feeder.mInstance.periodic();
      Indexer.mInstance.periodic();
    }
    assertTrue(feederSetpoint() > 0, "Manual is actually feeding");
    assertTrue(OperatorStatus.feedAllowed(), "panel should agree with active Manual target");
    m.end(true);
  }

  @Test
  void acceptancePanelWaitsForStableReady() {
    OperatorStatus.resetForTest();
    OperatorStatus.setClockForTest(Timer::getFPGATimestamp);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    Shooting s = new Shooting(false);
    s.initialize();
    stepWithPanel(s);
    stepWithPanel(s);
    stepWithPanel(s);
    assertEquals(0.0, feederSetpoint(), EPS, "still in startup debounce");
    assertFalse(OperatorStatus.feedAllowed(), "not yet permitted to feed");
    s.end(true);
  }

  @Test
  void acceptanceStopsOnFirstNotReadyLoopCurrentMode() {
    // Energy-save mode feeds with a 50 A current request; the same-loop stop must drop that
    // request, not just a velocity setpoint.
    Shooting s = new Shooting(true);
    s.initialize();
    boolean currentFeeding = false;
    for (int i = 0; i < 30 && !currentFeeding; i++) {
      step(s);
      currentFeeding =
          Feeder.mInstance.getControlMode() == MotorIO.ControlMode.CURRENT
              && Math.abs(Feeder.mInstance.getCurrentSetpointAmps() - 50.0) < EPS;
    }
    assertTrue(currentFeeding, "energy-save mode feeds with a 50 A current request");
    assertEquals(MotorIO.ControlMode.CURRENT, Indexer.mInstance.getControlMode());

    DashboardControlsTestUtil.set(FLYWHEEL, 0.0);
    step(s);
    assertEquals(
        MotorIO.ControlMode.VOLTAGE,
        Feeder.mInstance.getControlMode(),
        "feeder leaves CURRENT on the detection loop");
    assertEquals(0.0, Feeder.mInstance.getCurrentSetpointAmps(), EPS, "no current request left");
    assertEquals(0.0, feederSetpoint(), EPS);
    assertEquals(MotorIO.ControlMode.VOLTAGE, Indexer.mInstance.getControlMode());
    assertEquals(0.0, Indexer.mInstance.getCurrentSetpointAmps(), EPS);
    assertEquals(0.0, indexerSetpoint(), EPS);
    s.end(true);
  }

  @Test
  void acceptanceUnjamRetractFiresAfterRemainingTime() throws Exception {
    // Stretch the retract delay so the jam + back-off land inside it: without the freeze at UNJAM
    // entry the timer would outrun the delay during the back-off and RETRACT would fire instantly
    // on resume; with it, only the pre-jam feeding time counts.
    overrideTunable(ShootingConstants.RETRACT_DELAY_SEC, "Shooting/retract_delay", 0.5);
    try {
      Shooting s = new Shooting(false);
      s.initialize();
      assertTrue(stepUntilFeeding(s, 30), "reached SHOOT");

      Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(100.0);
      for (int i = 0; i < 30 && !stateOf(s).equals("UNJAM"); i++) {
        step(s);
      }
      assertEquals("UNJAM", stateOf(s));
      double frozenAt = retractElapsed(s);
      assertTrue(frozenAt < 0.5, "retract delay not yet reached when the jam fired");
      Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(null);

      for (int i = 0; i < 40 && stateOf(s).equals("UNJAM"); i++) {
        step(s);
      }
      assertEquals("PAUSED", stateOf(s));
      assertEquals(frozenAt, retractElapsed(s), EPS, "unjam time never counted");

      boolean resumed = false;
      for (int i = 0; i < 15 && !resumed; i++) {
        step(s);
        resumed = stateOf(s).equals("SHOOT");
      }
      assertTrue(resumed, "resumed after the post-unjam stable window");

      double retractCm = ShootingConstants.INTAKE_RETRACT_POSITION_CM.getAsDouble();
      for (int i = 0; i < 5; i++) {
        assertTrue(
            Math.abs(IntakeDeploy.mInstance.getPositionSetpointRot() - retractCm) > EPS,
            "no instant retract on resume, loop " + i);
        step(s);
      }
      boolean retracted = false;
      for (int i = 0; i < 40 && !retracted; i++) {
        step(s);
        retracted = Math.abs(IntakeDeploy.mInstance.getPositionSetpointRot() - retractCm) < EPS;
      }
      assertTrue(retracted, "retract fired after the remaining feed time");
      s.end(true);
    } finally {
      overrideTunable(ShootingConstants.RETRACT_DELAY_SEC, "Shooting/retract_delay", 0.2);
    }
  }

  @Test
  void acceptancePanelTracksControlLoopByLoop() throws Exception {
    OperatorStatus.resetForTest();
    OperatorStatus.setClockForTest(Timer::getFPGATimestamp);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    Shooting s = new Shooting(false);
    s.initialize();
    // Startup: the panel must never allow feeding before the control loop enters SHOOT, and while
    // the shot is raw-ready but still inside the debounce the reason is Stabilizing.
    boolean sawStabilizing = false;
    boolean reachedShoot = false;
    for (int i = 0; i < 30 && !reachedShoot; i++) {
      stepWithPanel(s);
      String st = stateOf(s);
      if (st.equals("SHOOT")) {
        reachedShoot = true;
        assertTrue(
            OperatorStatus.feedAllowed(), "panel allows on the same loop control enters SHOOT");
        assertEquals(ShotReadiness.REASON_NONE, OperatorStatus.feedBlockedReason());
      } else {
        assertEquals("SPIN_UP", st);
        assertFalse(OperatorStatus.feedAllowed(), "no early allow at startup, loop " + i);
        String reason = OperatorStatus.feedBlockedReason();
        assertTrue(
            reason.equals(ShotReadiness.REASON_STABILIZING)
                || reason.equals(ShotReadiness.REASON_DRUM_NOT_AT_SPEED),
            "startup loop " + i + " reason: " + reason);
        sawStabilizing |= reason.equals(ShotReadiness.REASON_STABILIZING);
      }
    }
    assertTrue(reachedShoot, "reached SHOOT");
    assertTrue(sawStabilizing, "the startup debounce phase shows Stabilizing");

    // Mid-volley loss: the stop and the panel decision land on the detection loop.
    DashboardControlsTestUtil.set(FLYWHEEL, 0.0);
    stepWithPanel(s);
    assertEquals("PAUSED", stateOf(s));
    assertEquals(0.0, feederSetpoint(), EPS, "feed stops on the detection loop");
    assertFalse(OperatorStatus.feedAllowed());
    assertEquals(ShotReadiness.REASON_FLYWHEEL_ZERO, OperatorStatus.feedBlockedReason());

    // Recovery: the panel waits out the same debounce the control loop does.
    DashboardControlsTestUtil.set(FLYWHEEL, 100.0);
    boolean resumed = false;
    boolean sawRecoveryStabilizing = false;
    for (int i = 0; i < 15 && !resumed; i++) {
      stepWithPanel(s);
      String st = stateOf(s);
      if (st.equals("SHOOT")) {
        resumed = true;
        assertTrue(OperatorStatus.feedAllowed(), "panel allows on the resume loop");
      } else {
        assertEquals("PAUSED", st);
        assertFalse(OperatorStatus.feedAllowed(), "no early allow during recovery, loop " + i);
        assertFalse(OperatorStatus.feedBlockedReason().isEmpty(), "a reason is always shown");
        sawRecoveryStabilizing |=
            OperatorStatus.feedBlockedReason().equals(ShotReadiness.REASON_STABILIZING);
      }
    }
    assertTrue(resumed, "auto-resumed");
    assertTrue(sawRecoveryStabilizing, "the recovery debounce phase shows Stabilizing");

    // Jam: while the jam debounce runs the volley keeps feeding and the panel agrees; in UNJAM the
    // panel shows the jam overlay.
    Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(100.0);
    boolean unjammed = false;
    for (int i = 0; i < 30 && !unjammed; i++) {
      stepWithPanel(s);
      // The jam debounce (0.2 s) and the retract delay (0.2 s) expire on the same loop here; which
      // feeding state the jam trips from depends on a one-loop test-clock lag, so accept both —
      // RETRACT is still part of the volley and keeps feeding.
      String st = stateOf(s);
      if (st.equals("UNJAM")) {
        unjammed = true;
      } else {
        assertTrue(
            st.equals("SHOOT") || st.equals("RETRACT"),
            "still in the volley, loop " + i + ": " + st);
        assertTrue(isFeeding(), "still feeding during the jam debounce, loop " + i);
        assertTrue(OperatorStatus.feedAllowed(), "loop " + i);
      }
    }
    assertTrue(unjammed, "UNJAM triggered");
    assertEquals(Action.CLEARING_JAM, OperatorStatus.currentAction());
    assertFalse(OperatorStatus.feedAllowed(), "jam clearing blocks feeding");
    assertEquals(ShotReadiness.REASON_CLEARING_JAM, OperatorStatus.feedBlockedReason());
    Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(null);
    boolean backOffOver = false;
    for (int i = 0; i < 40 && !backOffOver; i++) {
      stepWithPanel(s);
      if (stateOf(s).equals("UNJAM")) {
        assertFalse(OperatorStatus.feedAllowed(), "UNJAM loop " + i);
        assertEquals(ShotReadiness.REASON_CLEARING_JAM, OperatorStatus.feedBlockedReason());
      } else {
        backOffOver = true;
      }
    }
    assertTrue(backOffOver, "back-off ran to completion");

    // After the back-off the volley must re-earn a fresh stable window: never allowed while
    // PAUSED, allowed again on the resume loop. The CLEARING_JAM heartbeat stays fresh for its
    // 0.15 s expiry into the pause, so the panel reason is the jam overlay first, then the
    // command's own waiting decision.
    assertEquals("PAUSED", stateOf(s));
    assertFalse(OperatorStatus.feedAllowed(), "no feeding right after the back-off");
    resumed = false;
    for (int i = 0; i < 15 && !resumed; i++) {
      stepWithPanel(s);
      // The volley resumes into whichever feeding state the jam interrupted (SHOOT or RETRACT).
      String st = stateOf(s);
      if (st.equals("SHOOT") || st.equals("RETRACT")) {
        resumed = true;
        assertTrue(OperatorStatus.feedAllowed(), "panel allows on the post-unjam resume loop");
      } else {
        assertEquals("PAUSED", st);
        assertFalse(OperatorStatus.feedAllowed(), "post-unjam debounce loop " + i);
        String reason = OperatorStatus.feedBlockedReason();
        assertTrue(
            reason.equals(ShotReadiness.REASON_CLEARING_JAM)
                || reason.equals(ShotReadiness.REASON_STABILIZING),
            "post-unjam loop " + i + " reason: " + reason);
      }
    }
    assertTrue(resumed, "resumed after the post-unjam stable window");

    // Key released: the decision clears at end(), and once the action heartbeat expires the panel
    // falls back to its own preview — drum back at stow speed, not ready.
    s.end(true);
    for (int i = 0; i < 10; i++) {
      SimHooks.stepTiming(0.02);
      Drum.mInstance.periodic();
      Hood.mInstance.periodic();
    }
    OperatorStatus.update();
    assertEquals(Action.IDLE, OperatorStatus.currentAction());
    assertFalse(OperatorStatus.feedAllowed());
    assertEquals(ShotReadiness.REASON_DRUM_NOT_AT_SPEED, OperatorStatus.feedBlockedReason());
  }

  @Test
  void acceptancePanelFollowsActiveCommandSwitch() {
    OperatorStatus.resetForTest();
    OperatorStatus.setClockForTest(Timer::getFPGATimestamp);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    Shooting shooting = new Shooting(false);
    shooting.initialize();
    boolean feeding = false;
    for (int i = 0; i < 30 && !feeding; i++) {
      stepWithPanel(shooting);
      feeding = isFeeding();
    }
    assertTrue(feeding, "Shooting reached SHOOT");
    assertTrue(OperatorStatus.feedAllowed(), "panel follows Shooting");
    assertEquals(Action.FEEDING, OperatorStatus.currentAction());

    shooting.end(true);
    for (int i = 0; i < 10; i++) {
      SimHooks.stepTiming(0.02);
      Drum.mInstance.periodic();
      Hood.mInstance.periodic();
    }
    OperatorStatus.update();
    assertEquals(Action.IDLE, OperatorStatus.currentAction());
    assertFalse(OperatorStatus.feedAllowed(), "Shooting ended, drum back at stow speed");

    // Manual feeds at its own 50 rot/s setpoint — below the dashboard fallback's full-speed
    // target, so only the command's own reported decision can show allowed.
    var manual = new frc.robot.commands.ManualCommand.Manual();
    manual.initialize();
    feeding = false;
    for (int i = 0; i < 30 && !feeding; i++) {
      stepWithPanel(manual);
      feeding = feederSetpoint() > 0;
    }
    assertTrue(feeding, "Manual reached SHOOT");
    assertTrue(OperatorStatus.feedAllowed(), "panel follows the active Manual target");
    assertEquals(Action.FEEDING, OperatorStatus.currentAction());

    manual.end(true);
    for (int i = 0; i < 10; i++) {
      SimHooks.stepTiming(0.02);
      Drum.mInstance.periodic();
      Hood.mInstance.periodic();
    }
    OperatorStatus.update();
    assertEquals(Action.IDLE, OperatorStatus.currentAction());
    assertFalse(OperatorStatus.feedAllowed(), "Manual ended, decision cleared");
  }
}
