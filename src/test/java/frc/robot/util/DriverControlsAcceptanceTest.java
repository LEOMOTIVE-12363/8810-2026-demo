package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.DriverControls.Action;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Scheduler-level acceptance for the EJECT wiring shape RobotContainer carries: press and release
 * both recompute the latched reverse output from the controller in control, and the release command
 * runs while disabled. Mirrors the production wiring with a plain probe field so a failure here
 * isolates the binding layer (edges, gating, disabled scheduling) from the mechanism IO.
 *
 * <p>Triggers poll on the default scheduler's button loop and live for the whole JVM once
 * registered (same convention as DriverControlsTest): every test builds a fresh DriverControls with
 * a fresh probe field and only asserts on its own — stale triggers re-run dead probes, harmlessly.
 */
class DriverControlsAcceptanceTest {
  private static final int BUTTON_LB = 5;

  private double voltage;
  private DriverControls controls;

  @BeforeAll
  static void setUpHal() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
  }

  @BeforeEach
  void setUp() {
    CommandScheduler.getInstance().cancelAll();
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    voltage = 0;
    controls = new DriverControls();
    // Same command types, ordering and latched writes as the RobotContainer EJECT bindings.
    controls.onPress(Action.EJECT, () -> Commands.runOnce(() -> voltage = desiredEjectVolts()));
    controls.onRelease(
        Action.EJECT,
        () -> Commands.runOnce(() -> voltage = desiredEjectVolts()).ignoringDisable(true));
    controls.whileHeld(Action.EJECT, () -> Commands.run(() -> {}));
  }

  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    DriverStationSim.resetData();
    DriverStationSim.setEnabled(false);
    DriverStationSim.setDsAttached(false);
    DriverStationSim.notifyNewData();
  }

  /** Mirrors RobotContainer.applyReverseFeedDesired for the LB-only subset. */
  private double desiredEjectVolts() {
    return controls.isHeldByActiveController(Action.EJECT) && DriverStation.isEnabled() ? -3 : 0;
  }

  private static void loop() {
    DriverStationSim.notifyNewData();
    CommandScheduler.getInstance().run();
  }

  private static void connected(int port, boolean yes) {
    DriverStationSim.setJoystickIsXbox(port, yes);
    DriverStationSim.setJoystickAxisCount(port, yes ? 6 : 0);
    DriverStationSim.setJoystickButtonCount(port, yes ? 10 : 0);
    DriverStationSim.setJoystickPOVCount(port, yes ? 1 : 0);
    DriverStationSim.setJoystickPOV(port, 0, -1);
  }

  @Test
  void acceptanceNormalReleaseAndDisconnectStopReverse() {
    connected(DriverControls.DEMO_PORT, true);
    loop();
    DriverStationSim.setJoystickButton(DriverControls.DEMO_PORT, BUTTON_LB, true);
    loop();
    assertEquals(-3, voltage);
    DriverStationSim.setJoystickButton(DriverControls.DEMO_PORT, BUTTON_LB, false);
    loop();
    assertEquals(0, voltage);
    DriverStationSim.setJoystickButton(DriverControls.DEMO_PORT, BUTTON_LB, true);
    loop();
    connected(DriverControls.DEMO_PORT, false);
    loop();
    assertEquals(0, voltage);
  }

  @Test
  void acceptanceTakeoverKeepsNewOwnersReverse() {
    connected(DriverControls.FULL_PORT, true);
    loop();
    DriverStationSim.setJoystickButton(DriverControls.FULL_PORT, BUTTON_LB, true);
    loop();
    assertEquals(-3, voltage);
    connected(DriverControls.DEMO_PORT, true);
    DriverStationSim.setJoystickButton(DriverControls.DEMO_PORT, BUTTON_LB, true);
    loop();
    loop();
    assertEquals(DriverControls.DEMO_PORT, controls.activePort());
    assertEquals(
        -3, voltage, "new owner holds LB but the old owner's cleanup overwrote its reverse");
  }

  @Test
  void acceptanceFallbackKeepsNewOwnersReverse() {
    connected(DriverControls.FULL_PORT, true);
    connected(DriverControls.DEMO_PORT, true);
    loop();
    DriverStationSim.setJoystickButton(DriverControls.FULL_PORT, BUTTON_LB, true);
    DriverStationSim.setJoystickButton(DriverControls.DEMO_PORT, BUTTON_LB, true);
    loop();
    assertEquals(-3, voltage);
    connected(DriverControls.DEMO_PORT, false);
    loop();
    loop();
    assertEquals(DriverControls.FULL_PORT, controls.activePort());
    assertEquals(
        -3, voltage, "fallback controller holds LB but the losing source's cleanup overwrote it");
  }

  @Test
  void acceptanceDisabledDisconnectClearsReverseBeforeReenable() {
    connected(DriverControls.DEMO_PORT, true);
    loop();
    DriverStationSim.setJoystickButton(DriverControls.DEMO_PORT, BUTTON_LB, true);
    loop();
    assertEquals(-3, voltage);
    DriverStationSim.setEnabled(false);
    loop();
    connected(DriverControls.DEMO_PORT, false);
    loop();
    DriverStationSim.setEnabled(true);
    loop();
    assertEquals(-1, controls.activePort());
    assertEquals(
        0, voltage, "no controller is connected but the reverse request was never cleared");
  }
}
