package frc.robot.commands.ShootingCommand;

import static frc.robot.util.DashboardControlsTestUtil.FLYWHEEL;
import static frc.robot.util.DashboardControlsTestUtil.HOOD;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
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
import frc.robot.util.OperatorStatus;
import frc.robot.util.OperatorStatus.Action;
import frc.robot.util.ShotReadiness;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

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

    // Dashboard edits mid-volley still change the commanded setpoints. The readiness gate now
    // fires before prepareShooter on the transition loop (the new 5% target reads as not-ready
    // against the still-old measured speed), so the new setpoints land one loop later than
    // before, from PAUSED.
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
    step(shooting);
    assertEquals(0.0, feederSetpoint(), EPS, "feed path stopped within 2 loops");
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
    step(shooting);
    step(shooting);
    assertEquals(0.0, feederSetpoint(), EPS, "paused");

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
    step(shooting);
    assertEquals(0.0, feederSetpoint(), EPS, "feeding paused within 2 loops");

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
    // feed. (The sim drum never sags during the stall, so stable-ready passes almost immediately;
    // on the real robot the drum dip would hold PAUSED until it recovers.)
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
}
