package frc.robot.commands.ShootingCommand;

import static frc.robot.util.DashboardControlsTestUtil.FLYWHEEL;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIO;
import frc.robot.subsystems.Drum.Drum;
import frc.robot.subsystems.Drum.DrumConstants;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.IntakeDeploy.IntakeDeploy;
import frc.robot.subsystems.IntakeRoller.IntakeRoller;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.DashboardControlsTestUtil;
import frc.robot.util.OperatorStatus;
import java.lang.reflect.Field;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Readiness-loss flavors that change the MEASUREMENT, not the target: the drum sagging below its
 * unchanged setpoint and the hood drifting off its unchanged target. ShootingTest covers the
 * target-side flavors (dashboard edits, 0% limit); here the fault is injected into the logged
 * mechanism inputs right before the command's execute() — the same layer the jam-current override
 * uses — so the readiness rule sees a sagged/drifted measurement while the setpoint stays put.
 *
 * <p>Injection semantics: the sim mechanism IO is instant-setpoint (the measured state snaps to the
 * setpoint every periodic), so an injected value only exists for the execute() it precedes and is
 * cleared by the next periodic. Sustained faults are re-injected every loop; recovery happens on
 * the first loop without injection. This proves the gate's reaction to a measured fault and its
 * timing — NOT the real flywheel's spin-down/spin-up dynamics, which the instant-setpoint sim
 * cannot model.
 */
class ShootingMeasurementFaultTest {
  private static final double EPS = 1e-6;
  /** Well outside both tolerances: drum 1.0 rot/s, hood 0.02 rot. */
  private static final double DRUM_SAG_ROTPS = DrumConstants.MaxShotVelocity - 7.0;

  private static final double HOOD_DRIFT_ROT = 0.1;

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

  private static double feederSetpoint() {
    return Feeder.mInstance.getVelocitySetpointRotPerSec();
  }

  private static double indexerSetpoint() {
    return Indexer.mInstance.getVelocitySetpointRotPerSec();
  }

  private static boolean isFeeding() {
    return Math.abs(feederSetpoint() - ShootingConstants.FeederRotpsTunable.getAsDouble()) < EPS;
  }

  private static boolean isCurrentFeeding() {
    return Feeder.mInstance.getControlMode() == MotorIO.ControlMode.CURRENT
        && Math.abs(Feeder.mInstance.getCurrentSetpointAmps() - 50.0) < EPS;
  }

  private static boolean stepUntilFeeding(Shooting shooting, int maxLoops) {
    for (int i = 0; i < maxLoops; i++) {
      step(shooting);
      if (isFeeding()) {
        return true;
      }
    }
    return false;
  }

  private static boolean stepUntilCurrentFeeding(Shooting shooting, int maxLoops) {
    for (int i = 0; i < maxLoops; i++) {
      step(shooting);
      if (isCurrentFeeding()) {
        return true;
      }
    }
    return false;
  }

  private static String stateOf(Shooting s) throws Exception {
    Field f = Shooting.class.getDeclaredField("state");
    f.setAccessible(true);
    return f.get(s).toString();
  }

  /**
   * Overwrites one field of a subsystem's logged inputs, mirroring what a real measurement fault
   * would look like at the only layer the control loop reads. Must run AFTER a periodic (which
   * refreshes inputs from the IO) and BEFORE the execute() that should see the fault.
   */
  private static void overrideInput(MotorSubsystem subsystem, String field, double value)
      throws Exception {
    Field inputsField = MotorSubsystem.class.getDeclaredField("inputs");
    inputsField.setAccessible(true);
    Object inputs = inputsField.get(subsystem);
    for (Class<?> c = inputs.getClass(); c != null; c = c.getSuperclass()) {
      try {
        Field f = c.getDeclaredField(field);
        f.setAccessible(true);
        f.setDouble(inputs, value);
        return;
      } catch (NoSuchFieldException e) {
        // walk up to the declaring class
      }
    }
    throw new NoSuchFieldException(field);
  }

  private static void injectDrumVelocity(double rotPerSec) throws Exception {
    overrideInput(Drum.mInstance, "velocityRadPerSec", rotPerSec * 2.0 * Math.PI);
  }

  private static void injectHoodPosition(double rot) throws Exception {
    overrideInput(Hood.mInstance, "positionRad", rot * 2.0 * Math.PI);
  }

  @Test
  void measuredDrumSagStopsFeedOnDetectionLoopAndResumesAfterStableWindow() throws Exception {
    assertNull(Drive.mInstance, "test must run with no drivetrain / pose source");
    Shooting s = new Shooting(false);
    s.initialize();
    assertTrue(stepUntilFeeding(s, 30), "reached SHOOT");

    // The target never changes in this test: only the measured drum speed sags.
    double targetBefore = Drum.mInstance.getVelocitySetpointRotPerSec();
    injectDrumVelocity(DRUM_SAG_ROTPS);
    step(s);
    assertEquals(0.0, feederSetpoint(), EPS, "feed stopped on the detection loop");
    assertEquals(0.0, indexerSetpoint(), EPS, "indexer stopped too");
    assertEquals(0.0, IntakeRoller.mInstance.getVelocitySetpointRotPerSec(), EPS);
    assertEquals("PAUSED", stateOf(s));
    assertEquals(
        targetBefore,
        Drum.mInstance.getVelocitySetpointRotPerSec(),
        EPS,
        "drum keeps being driven to the unchanged target");

    for (int i = 0; i < 10; i++) {
      injectDrumVelocity(DRUM_SAG_ROTPS);
      step(s);
      assertEquals(0.0, feederSetpoint(), EPS, "sustained sag keeps the volley paused, loop " + i);
    }

    // Recovery: the injection stops, the next periodic already snapped the measurement back, so
    // the shot is raw-ready — but feeding must still wait out the full 0.15 s stable window.
    int loopsToResume = 0;
    for (int i = 0; i < 15 && !isFeeding(); i++) {
      step(s);
      loopsToResume++;
    }
    assertTrue(isFeeding(), "resumed feeding after the measured recovery");
    assertTrue(
        loopsToResume >= 7,
        "recovery needed the stable window, not the first good sample: " + loopsToResume);
    s.end(true);
  }

  @Test
  void measuredHoodDriftStopsFeedOnDetectionLoopWhileShooterKeepsTargets() throws Exception {
    Shooting s = new Shooting(false);
    s.initialize();
    assertTrue(stepUntilFeeding(s, 30), "reached SHOOT");

    double hoodTarget = Hood.mInstance.getPositionSetpointRot();
    injectHoodPosition(hoodTarget + HOOD_DRIFT_ROT);
    step(s);
    assertEquals(0.0, feederSetpoint(), EPS, "hood drift stops the feed on the detection loop");
    assertEquals("PAUSED", stateOf(s));
    assertEquals(
        hoodTarget,
        Hood.mInstance.getPositionSetpointRot(),
        EPS,
        "hood keeps being driven to the unchanged target");
    assertEquals(
        DrumConstants.MaxShotVelocity,
        Drum.mInstance.getVelocitySetpointRotPerSec(),
        EPS,
        "drum keeps being driven");

    boolean resumed = false;
    for (int i = 0; i < 15 && !resumed; i++) {
      step(s);
      resumed = isFeeding();
    }
    assertTrue(resumed, "resumed once the measured hood recovered through the stable window");
    s.end(true);
  }

  @Test
  void faultsInsideToleranceDoNotPauseTheVolley() throws Exception {
    // Just inside both tolerances (drum 1.0 rot/s, hood 0.02 rot): the rule uses strict
    // inequalities, so these must not stop feeding.
    Shooting s = new Shooting(false);
    s.initialize();
    assertTrue(stepUntilFeeding(s, 30), "reached SHOOT");

    injectDrumVelocity(
        DrumConstants.MaxShotVelocity - 0.5 * ShootingConstants.SHOOTER_VELOCITY_TOLERANCE_ROTPS);
    step(s);
    assertTrue(isFeeding(), "a drum fault inside the tolerance must not pause");

    double hoodTarget = Hood.mInstance.getPositionSetpointRot();
    injectHoodPosition(hoodTarget + 0.5 * ShootingConstants.HOOD_ANGLE_TOLERANCE_ROT);
    step(s);
    assertTrue(isFeeding(), "a hood fault inside the tolerance must not pause");
    s.end(true);
  }

  @Test
  void measuredDrumSagCurrentModeWithdrawsCurrentRequestAndRestoresIt() throws Exception {
    // Energy-save mode feeds with a 50 A current request; the same-loop stop must withdraw the
    // actual current request (control mode and amps), not just zero a velocity setpoint — and the
    // auto-resume must restore the current request, not fall back to velocity feed.
    Shooting s = new Shooting(true);
    s.initialize();
    assertTrue(stepUntilCurrentFeeding(s, 30), "energy-save mode feeding at 50 A");
    assertEquals(MotorIO.ControlMode.CURRENT, Indexer.mInstance.getControlMode());

    injectDrumVelocity(DRUM_SAG_ROTPS);
    step(s);
    assertEquals(
        MotorIO.ControlMode.VOLTAGE,
        Feeder.mInstance.getControlMode(),
        "feeder leaves CURRENT on the detection loop");
    assertEquals(0.0, Feeder.mInstance.getCurrentSetpointAmps(), EPS, "no current request left");
    assertEquals(0.0, Feeder.mInstance.getVoltageSetpoint(), EPS, "stopped, not reversed");
    assertEquals(MotorIO.ControlMode.VOLTAGE, Indexer.mInstance.getControlMode());
    assertEquals(0.0, Indexer.mInstance.getCurrentSetpointAmps(), EPS);
    assertEquals(0.0, indexerSetpoint(), EPS);

    int loopsToResume = 0;
    boolean restored = false;
    for (int i = 0; i < 15 && !restored; i++) {
      step(s);
      loopsToResume++;
      restored = isCurrentFeeding();
    }
    assertTrue(restored, "current-feed request restored after the stable window");
    assertTrue(
        loopsToResume >= 7, "current-mode recovery needed the stable window: " + loopsToResume);
    s.end(true);
  }

  @Test
  void pauseDuringRetractResumesIntoRetractAndKeepsIntakeRetracted() throws Exception {
    // The volley pauses from RETRACT too (not only SHOOT): the pause must keep pulling the intake
    // in, and the resume must return to RETRACT — never back to SHOOT, which would re-extend the
    // feed window semantics.
    Shooting s = new Shooting(false);
    s.initialize();
    assertTrue(stepUntilFeeding(s, 30), "reached SHOOT");
    for (int i = 0; i < 30 && !stateOf(s).equals("RETRACT"); i++) {
      step(s);
    }
    assertEquals("RETRACT", stateOf(s), "the 0.2 s retract delay elapsed");
    // The loop that flips the state to RETRACT still runs SHOOT's outputs (which never touch the
    // deploy setpoint); the retract position write lands one loop later.
    step(s);
    step(s);
    double retractCm = ShootingConstants.INTAKE_RETRACT_POSITION_CM.getAsDouble();
    assertEquals(
        retractCm, IntakeDeploy.mInstance.getPositionSetpointRot(), EPS, "intake retracting");

    DashboardControlsTestUtil.set(FLYWHEEL, 0.0);
    step(s);
    assertEquals("PAUSED", stateOf(s));
    assertEquals(0.0, feederSetpoint(), EPS, "feed stopped on the detection loop");
    for (int i = 0; i < 10; i++) {
      step(s);
      assertEquals(
          retractCm,
          IntakeDeploy.mInstance.getPositionSetpointRot(),
          EPS,
          "paused RETRACT keeps the intake retracted, loop " + i);
    }

    DashboardControlsTestUtil.set(FLYWHEEL, 100.0);
    boolean resumed = false;
    for (int i = 0; i < 15 && !resumed; i++) {
      step(s);
      resumed = isFeeding();
    }
    assertTrue(resumed, "resumed feeding");
    assertEquals("RETRACT", stateOf(s), "resume returns to the remembered RETRACT, not SHOOT");
    s.end(true);
  }

  @Test
  void timerAndDebouncerClocksAdvanceTogetherUnderSimHooks() {
    // Documents the test-clock relationship behind every timing assertion in the shooting tests:
    // Timer (RobotController.getTime) backs Timer.hasElapsed; MathSharedStore.getTimestamp backs
    // Debouncer. Both must advance with SimHooks.stepTiming or the debounce windows could not run
    // at all in-JVM. Any constant offset between them is fine (it shifts both anchors equally);
    // print it so the acceptance report can cite the measured value.
    double timer0 = Timer.getFPGATimestamp();
    double shared0 = MathSharedStore.getTimestamp();
    SimHooks.stepTiming(0.02);
    double timer1 = Timer.getFPGATimestamp();
    double shared1 = MathSharedStore.getTimestamp();
    assertEquals(0.02, timer1 - timer0, 1e-6, "Timer clock advanced by the step");
    assertEquals(0.02, shared1 - shared0, 1e-6, "Debouncer clock advanced by the same step");
    System.out.println(
        "[clock-diagnostic] Timer.getFPGATimestamp - MathSharedStore.getTimestamp = "
            + (timer1 - shared1)
            + " s");
  }
}
