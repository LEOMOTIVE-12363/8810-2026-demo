package frc.robot.commands.ManualCommand;

import static frc.robot.util.DashboardControlsTestUtil.FLYWHEEL;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.fail;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.subsystems.Drum.Drum;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.IntakeDeploy.IntakeDeploy;
import frc.robot.subsystems.IntakeRoller.IntakeRoller;
import frc.robot.util.DashboardControlsTestUtil;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Runs Manual against the sim mechanism IO. The Drum/Hood/feed-path singletons are JVM-shared
 * across test classes, so every test lets the command drive its known Manual setpoints and settle
 * before asserting.
 */
class ManualTest {
  private static final double EPS = 1e-6;

  @BeforeAll
  static void setUpHal() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
  }

  @AfterEach
  void resetDashboard() {
    DashboardControlsTestUtil.resetDefaults();
  }

  /** One robot loop: run the command, advance time, then update mechanism inputs. */
  private static void step(Manual manual) {
    manual.execute();
    SimHooks.stepTiming(0.02);
    runPeriodics();
  }

  /** Updates mechanism inputs without running the command (for assertions after end()). */
  private static void runPeriodics() {
    Drum.mInstance.periodic();
    Hood.mInstance.periodic();
    Feeder.mInstance.periodic();
    Indexer.mInstance.periodic();
    IntakeDeploy.mInstance.periodic();
    IntakeRoller.mInstance.periodic();
  }

  /** Steps until the feeder is commanded to feed (SHOOT reached); fails after maxLoops. */
  private static void stepUntilFeeding(Manual manual, int maxLoops) {
    for (int i = 0; i < maxLoops; i++) {
      step(manual);
      if (Feeder.mInstance.getVelocitySetpointRotPerSec() > 0.0) {
        return;
      }
    }
    fail("Manual never reached SHOOT within " + maxLoops + " loops");
  }

  @Test
  void feedsOnlyAfterStableReady() {
    Manual manual = new Manual();
    manual.initialize();

    // The feed path stays stopped while the shared 0.15 s readiness debounce runs.
    for (int i = 0; i < 3; i++) {
      step(manual);
      assertEquals(0.0, Feeder.mInstance.getVelocitySetpointRotPerSec(), EPS);
    }

    // Past the debounce the drum/hood are on target, SHOOT is reached, and the feeder runs.
    for (int i = 0; i < 12; i++) {
      step(manual);
    }
    assertEquals(
        ManualConstants.FeederRotpsTunable.getAsDouble(),
        Feeder.mInstance.getVelocitySetpointRotPerSec(),
        EPS);
    manual.end(true);
  }

  @Test
  void zeroFlywheelMidShootReturnsToPrepareAndResumes() {
    Manual manual = new Manual();
    manual.initialize();
    stepUntilFeeding(manual, 50);

    // Dropping the flywheel limit to 0% mid-SHOOT stops the feed path on the detection loop: the
    // raw-not-ready evaluation falls back to PREPARE and commands the stops in the same loop.
    DashboardControlsTestUtil.set(FLYWHEEL, 0.0);
    step(manual);
    assertEquals(0.0, Feeder.mInstance.getVelocitySetpointRotPerSec(), EPS);
    assertEquals(0.0, Drum.mInstance.getVelocitySetpointRotPerSec(), EPS);

    // The pause holds without feeding.
    for (int i = 0; i < 10; i++) {
      step(manual);
    }
    assertEquals(0.0, Feeder.mInstance.getVelocitySetpointRotPerSec(), EPS);

    // Restoring the limit re-enters SHOOT on its own after the debounce, with no end/initialize.
    DashboardControlsTestUtil.set(FLYWHEEL, 100.0);
    for (int i = 0; i < 15; i++) {
      step(manual);
    }
    assertEquals(
        ManualConstants.FeederRotpsTunable.getAsDouble(),
        Feeder.mInstance.getVelocitySetpointRotPerSec(),
        EPS);
    manual.end(true);
  }

  @Test
  void endStopsEverything() {
    Manual manual = new Manual();
    manual.initialize();
    stepUntilFeeding(manual, 50);

    manual.end(true);
    runPeriodics();

    assertEquals(0.0, Feeder.mInstance.getVelocitySetpointRotPerSec(), EPS);
    assertEquals(0.0, Indexer.mInstance.getVelocitySetpointRotPerSec(), EPS);
  }

  @Test
  void acceptanceManualStopsOnFirstNotReadyLoop() {
    Manual manual = new Manual();
    manual.initialize();
    stepUntilFeeding(manual, 50);

    DashboardControlsTestUtil.set(FLYWHEEL, 0.0);
    step(manual);
    assertEquals(
        0.0,
        Feeder.mInstance.getVelocitySetpointRotPerSec(),
        EPS,
        "feeder must stop on the detection loop");
    assertEquals(
        0.0,
        Indexer.mInstance.getVelocitySetpointRotPerSec(),
        EPS,
        "indexer must stop on the detection loop");
    assertEquals(
        0.0,
        IntakeRoller.mInstance.getVelocitySetpointRotPerSec(),
        EPS,
        "intake roller must stop on the detection loop");
    manual.end(true);
  }
}
