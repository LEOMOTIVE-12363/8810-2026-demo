package frc.robot.util;

import static frc.robot.util.DashboardControlsTestUtil.FLYWHEEL;
import static frc.robot.util.DashboardControlsTestUtil.HOOD;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.subsystems.Drum.Drum;
import frc.robot.subsystems.Drum.DrumConstants;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Hood.HoodConstants;
import frc.robot.util.OperatorStatus.Action;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * OperatorStatus heartbeat and priority rules with an injected clock (same pattern as
 * BallFlowTrackerTest): expiry always falls back to IDLE — a released key, an interrupted command,
 * or a disable simply stops reporting — and among fresh reports the highest priority wins. The
 * blocked-reason layering is checked with DriverStationSim, explicitly reset to disabled after
 * every test so the enabled state cannot leak into other test classes.
 */
class OperatorStatusTest {
  private final double[] now = {0.0};

  @BeforeAll
  static void setUpHal() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
  }

  @BeforeEach
  void injectClock() {
    OperatorStatus.resetForTest();
    OperatorStatus.setClockForTest(() -> now[0]);
    // Explicit disabled baseline (isEnabled() is enabled && dsAttached; both default false).
    DriverStationSim.setEnabled(false);
    DriverStationSim.setDsAttached(false);
    DriverStationSim.notifyNewData();
  }

  @AfterEach
  void resetStatus() {
    OperatorStatus.resetForTest();
    OperatorStatus.setClockForTest(null);
    DriverStationSim.setEnabled(false);
    DriverStationSim.setDsAttached(false);
    DriverStationSim.notifyNewData();
    DashboardControlsTestUtil.resetDefaults();
  }

  private void advanceTo(double t) {
    now[0] = t;
  }

  /** One robot loop for the sim mechanisms: advance time, then update mechanism inputs. */
  private static void stepMechanisms() {
    SimHooks.stepTiming(0.02);
    Drum.mInstance.periodic();
    Hood.mInstance.periodic();
  }

  @Test
  void noReportsIsIdle() {
    assertEquals(Action.IDLE, OperatorStatus.currentAction());
  }

  @Test
  void reportsExpireAfter150ms() {
    OperatorStatus.report(Action.FEEDING);
    assertEquals(Action.FEEDING, OperatorStatus.currentAction());
    advanceTo(0.14);
    assertEquals(Action.FEEDING, OperatorStatus.currentAction(), "still within the expiry");
    advanceTo(0.16);
    assertEquals(Action.IDLE, OperatorStatus.currentAction(), "heartbeat expired");
  }

  @Test
  void jamOutranksFeedUntilItExpires() {
    OperatorStatus.report(Action.FEEDING);
    OperatorStatus.report(Action.CLEARING_JAM);
    assertEquals(Action.CLEARING_JAM, OperatorStatus.currentAction());

    advanceTo(0.10);
    OperatorStatus.report(Action.FEEDING); // feeding keeps heartbeating, jam handling stopped
    advanceTo(0.16); // the jam heartbeat (t=0) expired; feed (t=0.10) is still fresh
    assertEquals(Action.FEEDING, OperatorStatus.currentAction());
  }

  @Test
  void highestPriorityFreshReportWins() {
    OperatorStatus.report(Action.INTAKING);
    OperatorStatus.report(Action.SPINNING_UP);
    OperatorStatus.report(Action.FEEDING);
    OperatorStatus.report(Action.WAITING_RECOVERY);
    assertEquals(Action.WAITING_RECOVERY, OperatorStatus.currentAction());
    OperatorStatus.report(Action.CLEARING_JAM);
    assertEquals(Action.CLEARING_JAM, OperatorStatus.currentAction());
    OperatorStatus.report(Action.REVERSING_LOCAL);
    assertEquals(Action.REVERSING_LOCAL, OperatorStatus.currentAction());
    OperatorStatus.report(Action.REVERSING_FULL);
    assertEquals(Action.REVERSING_FULL, OperatorStatus.currentAction());
  }

  @Test
  void disabledBlocksFeedRegardlessOfReports() {
    OperatorStatus.report(Action.FEEDING);
    OperatorStatus.update();
    assertEquals(Action.FEEDING, OperatorStatus.currentAction());
    assertEquals(ShotReadiness.REASON_DISABLED, OperatorStatus.feedBlockedReason());
    assertFalse(OperatorStatus.feedAllowed());
  }

  @Test
  void clearingJamBlocksAndExpiredJamWithReadyShotAllows() {
    // DriverStation.isEnabled() is enabled && dsAttached; notifyNewData() refreshes the cache.
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    OperatorStatus.report(Action.CLEARING_JAM);
    assertEquals(Action.CLEARING_JAM, OperatorStatus.currentAction());
    assertEquals(ShotReadiness.REASON_CLEARING_JAM, OperatorStatus.feedBlockedReason());
    assertFalse(OperatorStatus.feedAllowed());

    // Make the shot genuinely ready: flywheel 100%, hood entry matching the settled position.
    DashboardControlsTestUtil.set(FLYWHEEL, 100.0);
    DashboardControlsTestUtil.set(HOOD, 100.0);
    Drum.mInstance.setVelocityRotPerSec(DrumConstants.MaxShotVelocity);
    Hood.mInstance.setPositionRot(HoodConstants.MaxAngleDeg / 360.0);
    stepMechanisms();
    stepMechanisms();

    // The jam heartbeat expired with no re-report — the panel recovers on its own.
    advanceTo(0.20);
    OperatorStatus.update();
    assertEquals(Action.IDLE, OperatorStatus.currentAction());
    assertTrue(OperatorStatus.feedAllowed());
    assertEquals(ShotReadiness.REASON_NONE, OperatorStatus.feedBlockedReason());
  }

  @Test
  void publishWritesTheRobotActionKey() {
    OperatorStatus.report(Action.INTAKING);
    OperatorStatus.publish(0);
    assertEquals(
        "Intaking",
        NetworkTableInstance.getDefault().getEntry("/SmartDashboard/Robot Action").getString(""));
  }
}
