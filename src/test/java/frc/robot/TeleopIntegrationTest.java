package frc.robot;

import static frc.robot.util.DashboardControlsTestUtil.FLYWHEEL;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.io.MotorIO;
import frc.robot.commands.ManualCommand.ManualConstants;
import frc.robot.commands.ShootingCommand.ShootingConstants;
import frc.robot.simulation.MapleSimArena;
import frc.robot.subsystems.Drum.Drum;
import frc.robot.subsystems.Drum.DrumConstants;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.IntakeDeploy.IntakeDeploy;
import frc.robot.subsystems.IntakeRoller.IntakeRoller;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.DashboardControlsTestUtil;
import frc.robot.util.OperatorStatus;
import frc.robot.util.ShotReadiness;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Full teleop integration: the REAL RobotContainer (its bindings, default drive command, and
 * singletons), the real CommandScheduler, DriverStationSim joystick/alliance/enable inputs, and the
 * maple-sim physics arena — the same wiring {@code Robot} runs on the demo robot, minus the
 * LoggedRobot wrapper.
 *
 * <p>Each {@link #stepRobot()} mirrors one Robot loop: DS refresh → scheduler (buttons, subsystem
 * periodics, commands) → operator-panel update → 10 Hz dashboard publish (every 5th loop) → 20 ms
 * clock step → maple-sim physics advance. Dashboard edits go through real NetworkTables entries
 * (the same path an Elastic slider takes).
 *
 * <p>Timing note: inside one scheduler loop the subsystem periodics read the mechanism inputs
 * BEFORE this loop's command writes land, so an output change an edge command makes (end() stops,
 * press writes) is observable one loop later — every edge helper here follows with a settle loop
 * before assertions, which still fails if the command kept feeding one loop longer than allowed.
 *
 * <p>Singleton/binding hygiene: this class is the only test that constructs RobotContainer (the
 * reverse-path acceptance classes construct their own — see
 * DriverControlsIntegrationAcceptanceTest). This class's {@code @AfterAll} nulls {@code
 * Drive.mInstance} (ShootingTest requires it null), unregisters the drive, and swaps the
 * scheduler's active button loop so the container's live bindings cannot leak into other classes.
 */
class TeleopIntegrationTest {
  private static final double EPS = 1e-6;
  // Xbox HID: buttons A=1 B=2 X=3 Y=4 LB=5 RB=6; axes 0=leftX 1=leftY 2=leftTrigger 4=rightX.
  private static final int BUTTON_A = 1;
  private static final int BUTTON_B = 2;
  private static final int BUTTON_X = 3;
  private static final int BUTTON_Y = 4;
  private static final int BUTTON_LB = 5;
  private static final int BUTTON_RB = 6;
  private static final int AXIS_LEFT_X = 0;
  private static final int AXIS_LEFT_Y = 1;
  private static final int AXIS_LEFT_TRIGGER = 2;
  private static final int AXIS_RIGHT_X = 4;
  private static final int POV_INDEX = 0;

  private static RobotContainer container;
  private static int dashboardCounter;

  @BeforeAll
  static void setUp() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
    DriverStationSim.setAllianceStationId(AllianceStationID.Unknown);
    container = new RobotContainer();
    assertNotNull(Drive.mInstance, "container must build the drivetrain");
  }

  @AfterAll
  static void tearDown() {
    CommandScheduler scheduler = CommandScheduler.getInstance();
    scheduler.cancelAll();
    if (Drive.mInstance != null) {
      scheduler.unregisterSubsystem(Drive.mInstance);
      Drive.mInstance = null; // ShootingTest asserts the singleton is null outside this class
    }
    // Triggers always bind to the scheduler's DEFAULT button loop (see the Trigger constructor),
    // which this class had been polling normally — nothing was swapped at setup. Keep the active
    // loop pointing back at the default so later classes' fresh Trigger() bindings are polled
    // again; the container's own bindings re-arm there too, which is the same live-container
    // condition the other binding test classes already run under (same-command writes are
    // idempotent recomputes, and none of them assert on this container's motors).
    scheduler.setActiveButtonLoop(scheduler.getDefaultButtonLoop());
    resetJoystick(0);
    resetJoystick(1);
    DriverStationSim.setEnabled(false);
    DriverStationSim.setDsAttached(false);
    DriverStationSim.setAllianceStationId(AllianceStationID.Unknown);
    DriverStationSim.notifyNewData();
    DashboardControlsTestUtil.resetDefaults();
    OperatorStatus.resetForTest();
    OperatorStatus.setClockForTest(null);
  }

  @BeforeEach
  void resetState() {
    CommandScheduler.getInstance().cancelAll();
    resetJoystick(0);
    resetJoystick(1);
    DriverStationSim.setAllianceStationId(AllianceStationID.Unknown);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    DashboardControlsTestUtil.resetDefaults();
    OperatorStatus.resetForTest();
    OperatorStatus.setClockForTest(null);
    Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(null);
    Drive.mInstance.setPose(new Pose2d(3.0, 2.0, Rotation2d.kZero));
    for (int i = 0; i < 5; i++) {
      stepRobot(); // let cancelled commands' end() outputs reach the mechanism sim
    }
  }

  /** One Robot loop, in Robot.robotPeriodic order (see class doc). */
  private static void stepRobot() {
    DriverStationSim.notifyNewData();
    CommandScheduler.getInstance().run(); // buttons + subsystem periodics + commands
    OperatorStatus.update();
    if (++dashboardCounter >= 5) {
      dashboardCounter = 0;
      container.updateDashboardOutputs();
      OperatorStatus.publish(1);
    }
    SimHooks.stepTiming(0.02);
    MapleSimArena.getInstance().simulationPeriodic();
  }

  /** Extra loop so edge-command writes reach the logged mechanism inputs (see class doc). */
  private static void settle() {
    stepRobot();
  }

  /** Runs until the 10 Hz dashboard publish block has fired at least once. */
  private static void settleToPublish() {
    for (int i = 0; i < 6; i++) {
      stepRobot();
    }
  }

  private static void connect(int port) {
    DriverStationSim.setJoystickIsXbox(port, true);
    DriverStationSim.setJoystickAxisCount(port, 6);
    DriverStationSim.setJoystickButtonCount(port, 10);
    DriverStationSim.setJoystickPOVCount(port, 1);
    stepRobot();
  }

  private static void disconnect(int port) {
    resetJoystick(port);
    stepRobot();
  }

  private static void resetJoystick(int port) {
    neutralizeJoystick(port);
    DriverStationSim.setJoystickIsXbox(port, false);
    DriverStationSim.setJoystickAxisCount(port, 0);
    DriverStationSim.setJoystickButtonCount(port, 0);
    DriverStationSim.setJoystickPOVCount(port, 0);
  }

  private static void neutralizeJoystick(int port) {
    for (int axis = 0; axis < 6; axis++) {
      DriverStationSim.setJoystickAxis(port, axis, 0.0);
    }
    for (int button = 1; button <= 10; button++) {
      DriverStationSim.setJoystickButton(port, button, false);
    }
    DriverStationSim.setJoystickPOV(port, POV_INDEX, -1);
  }

  private static void press(int port, int button) {
    DriverStationSim.setJoystickButton(port, button, true);
    stepRobot();
  }

  private static void release(int port, int button) {
    DriverStationSim.setJoystickButton(port, button, false);
    stepRobot();
  }

  private static void setAxis(int port, int axis, double value) {
    DriverStationSim.setJoystickAxis(port, axis, value);
    stepRobot();
  }

  private static void setTrigger(int port, double value) {
    DriverStationSim.setJoystickAxis(port, AXIS_LEFT_TRIGGER, value);
    stepRobot();
  }

  private static void setPov(int port, int degrees) {
    DriverStationSim.setJoystickPOV(port, POV_INDEX, degrees);
    stepRobot();
  }

  private static void setAlliance(AllianceStationID station) {
    DriverStationSim.setAllianceStationId(station);
    DriverStationSim.notifyNewData();
  }

  /**
   * Settles the module steer angles to the straight-ahead state (small forward blip, then brake).
   * Open-loop swerve has no heading hold, and a lateral command from an arbitrary initial wheel
   * angle produces a large transient heading veer in the sim; starting from aligned wheels keeps
   * the direction assertions deterministic. The veer itself is a sim/physics observation, recorded
   * in the acceptance report — the template's joystick drive has no closed-loop heading on real
   * hardware either.
   */
  private static void alignWheelsForward() {
    DriverStationSim.setJoystickAxis(1, AXIS_LEFT_Y, -0.3);
    for (int i = 0; i < 8; i++) stepRobot();
    DriverStationSim.setJoystickAxis(1, AXIS_LEFT_Y, 0.0);
    for (int i = 0; i < 15; i++) stepRobot();
  }

  private static void teleport(Pose2d pose) {
    Drive.mInstance.setPose(pose);
    for (int i = 0; i < 3; i++) stepRobot(); // sync maple body + odometry
    for (int i = 0; i < 20; i++) {
      stepRobot(); // brake out any residual chassis velocity from the previous test
    }
  }

  private static Pose2d pose() {
    return Drive.mInstance.getPose();
  }

  private static ChassisSpeeds speeds() {
    return Drive.mInstance.getChassisSpeeds();
  }

  private interface SpeedComponent {
    double get(ChassisSpeeds speeds);
  }

  /** Runs {@code loops} steps; returns the mean measured component over the final 5 loops. */
  private static double mean(int loops, SpeedComponent component) {
    double sum = 0;
    int samples = 0;
    for (int i = 0; i < loops; i++) {
      stepRobot();
      if (i >= loops - 5) {
        sum += component.get(speeds());
        samples++;
      }
    }
    return sum / samples;
  }

  private static double meanVx(int loops) {
    return mean(loops, s -> Math.abs(s.vxMetersPerSecond));
  }

  private static double meanVy(int loops) {
    return mean(loops, s -> Math.abs(s.vyMetersPerSecond));
  }

  private static double meanOmega(int loops) {
    return mean(loops, s -> Math.abs(s.omegaRadiansPerSecond));
  }

  /** Signed mean omega over a short window (no 2*pi wrap ambiguity). */
  private static double signedOmega(int loops) {
    return mean(loops, s -> s.omegaRadiansPerSecond);
  }

  private static double feederSetpoint() {
    return Feeder.mInstance.getVelocitySetpointRotPerSec();
  }

  private static double feederAmps() {
    return Feeder.mInstance.getCurrentSetpointAmps();
  }

  private static boolean stepUntilFeeding(int maxLoops) {
    for (int i = 0; i < maxLoops; i++) {
      stepRobot();
      if (Math.abs(feederSetpoint() - ShootingConstants.FeederRotpsTunable.getAsDouble()) < EPS) {
        return true;
      }
    }
    return false;
  }

  private static boolean stepUntilManualFeeding(int maxLoops) {
    for (int i = 0; i < maxLoops; i++) {
      stepRobot();
      if (Math.abs(feederSetpoint() - ManualConstants.FeederRotpsTunable.getAsDouble()) < EPS) {
        return true;
      }
    }
    return false;
  }

  private static String robotAction() {
    return NetworkTableInstance.getDefault()
        .getEntry("/SmartDashboard/Robot Action")
        .getString("-");
  }

  private static boolean feedAllowedKey() {
    return NetworkTableInstance.getDefault()
        .getEntry("/SmartDashboard/Feed Allowed")
        .getBoolean(true);
  }

  private static String feedBlockedReasonKey() {
    return NetworkTableInstance.getDefault()
        .getEntry("/SmartDashboard/Feed Blocked Reason")
        .getString("-");
  }

  // ---------------------------------------------------------------------------
  // Item 2: drive directions, deadband, scaling, alliance flip, heading reset
  // ---------------------------------------------------------------------------

  @Test
  void blueAllianceForwardStrafeAndRotateDirections() {
    setAlliance(AllianceStationID.Blue1);
    connect(1);

    // Stick forward (leftY = -1): field-relative +X motion, heading held near zero.
    teleport(new Pose2d(3.0, 2.0, Rotation2d.kZero));
    setAxis(1, AXIS_LEFT_Y, -1.0);
    double vx = meanVx(80);
    assertTrue(vx > 1.0, "forward drives +X, measured |vx|=" + vx);
    assertTrue(pose().getX() > 3.5, "pose X grew, now " + pose().getX());
    assertTrue(Math.abs(pose().getY() - 2.0) < 0.8, "forward does not drift Y: " + pose().getY());
    assertTrue(Math.abs(pose().getRotation().getDegrees()) < 15, "heading held");
    setAxis(1, AXIS_LEFT_Y, 0.0);

    // Stick right (leftX = +1): field -Y motion. x = 2.5 keeps clear of the blue HUB (x >= 3.6).
    teleport(new Pose2d(2.5, 6.0, Rotation2d.kZero));
    setAxis(1, AXIS_LEFT_X, 1.0);
    double vy = meanVy(70);
    assertTrue(vy > 1.0, "strafe right drives -Y, measured |vy|=" + vy);
    assertTrue(pose().getY() < 5.5, "pose Y fell, now " + pose().getY());
    setAxis(1, AXIS_LEFT_X, 0.0);

    // Right stick right: negative omega (clockwise). Rotation2d.minus keeps the delta wrap-safe.
    teleport(new Pose2d(3.0, 2.0, Rotation2d.kZero));
    setAxis(1, AXIS_RIGHT_X, 1.0);
    double omega = signedOmega(25);
    assertTrue(omega < -1.0, "right stick commands clockwise (negative) omega: " + omega);
    Rotation2d headingBefore = pose().getRotation();
    for (int i = 0; i < 10; i++) stepRobot();
    assertTrue(
        pose().getRotation().minus(headingBefore).getRadians() < -0.5,
        "heading decreased, from " + headingBefore + " to " + pose().getRotation());
    setAxis(1, AXIS_RIGHT_X, 0.0);
  }

  @Test
  void blueAllianceForwardAtNinetyDegreesStrafesTowardPlusX() {
    // Field-relative: stick forward is always field +X on blue, regardless of robot heading. At a
    // 90° heading the robot therefore strafes toward +X.
    setAlliance(AllianceStationID.Blue1);
    connect(1);
    // Lane y = 2.0 is a clear corridor: below the blue HUB (y >= 3.0) and above the lower trench.
    teleport(new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(90)));

    alignWheelsForward();
    setAxis(1, AXIS_LEFT_Y, -1.0);
    double startY = pose().getY();
    for (int i = 0; i < 60; i++) stepRobot();
    assertTrue(
        pose().getX() - 3.0 > 0.5, "90° heading + forward = +X field motion, pose " + pose());
    assertTrue(Math.abs(pose().getY() - startY) < 0.8, "Y barely changes at 90°: " + pose());
    assertTrue(Math.abs(pose().getRotation().getDegrees() - 90.0) < 45, "no wild spin: " + pose());
    setAxis(1, AXIS_LEFT_Y, 0.0);
  }

  @Test
  void redAllianceFlipsTranslationButNotRotation() {
    setAlliance(AllianceStationID.Red1);
    connect(1);

    // The same physical stick-forward must now drive field -X (exactly one 180° flip).
    teleport(new Pose2d(8.0, 2.0, Rotation2d.kZero));
    setAxis(1, AXIS_LEFT_Y, -1.0);
    double vx = meanVx(70);
    assertTrue(vx > 1.0, "forward still moves the robot (flipped), |vx|=" + vx);
    assertTrue(pose().getX() < 7.5, "red: forward drives -X, pose X now " + pose().getX());
    setAxis(1, AXIS_LEFT_Y, 0.0);

    // Same flip at a 90° heading: forward is field -X, so the robot strafes toward -X. Start at
    // x = 6 so the run stays on the field; lane y = 2.0 is clear of the HUB and the trenches.
    teleport(new Pose2d(6.0, 2.0, Rotation2d.fromDegrees(90)));
    alignWheelsForward();
    setAxis(1, AXIS_LEFT_Y, -1.0);
    double startY = pose().getY();
    for (int i = 0; i < 50; i++) stepRobot();
    assertTrue(
        6.0 - pose().getX() > 0.5, "red + 90° heading: forward = -X field motion, pose " + pose());
    assertTrue(Math.abs(pose().getY() - startY) < 0.8, "Y barely changes: " + pose());
    setAxis(1, AXIS_LEFT_Y, 0.0);

    // Rotation is never flipped: right stick still commands clockwise (negative) omega.
    teleport(new Pose2d(3.0, 2.0, Rotation2d.kZero));
    setAxis(1, AXIS_RIGHT_X, 1.0);
    double omega = signedOmega(25);
    assertTrue(omega < -1.0, "rotation unaffected by alliance, omega=" + omega);
    Rotation2d headingBefore = pose().getRotation();
    for (int i = 0; i < 10; i++) stepRobot();
    assertTrue(
        pose().getRotation().minus(headingBefore).getRadians() < -0.5,
        "heading still decreases on red, from " + headingBefore + " to " + pose().getRotation());
    setAxis(1, AXIS_RIGHT_X, 0.0);
  }

  @Test
  void unknownAllianceBehavesLikeBlue() {
    setAlliance(AllianceStationID.Unknown);
    connect(1);
    teleport(new Pose2d(3.0, 2.0, Rotation2d.kZero));

    setAxis(1, AXIS_LEFT_Y, -1.0);
    for (int i = 0; i < 60; i++) stepRobot();
    assertTrue(pose().getX() > 3.5, "no alliance info: forward = +X like blue, pose " + pose());
    setAxis(1, AXIS_LEFT_Y, 0.0);
  }

  @Test
  void subDeadbandInputCommandsZeroMotion() {
    setAlliance(AllianceStationID.Blue1);
    connect(1);
    teleport(new Pose2d(3.0, 2.0, Rotation2d.kZero));

    // 0.02 is below the 0.03 deadband on every axis at once.
    setAxis(1, AXIS_LEFT_X, 0.02);
    setAxis(1, AXIS_LEFT_Y, -0.02);
    setAxis(1, AXIS_RIGHT_X, 0.02);
    for (int i = 0; i < 50; i++) stepRobot();
    Pose2d after = pose();
    assertEquals(3.0, after.getX(), 0.08, "deadband keeps X: " + after);
    assertEquals(2.0, after.getY(), 0.08, "deadband keeps Y: " + after);
    assertEquals(0.0, after.getRotation().getRadians(), 0.05, "deadband keeps heading");
  }

  @Test
  void releasingTheStickBringsTheRobotToRest() {
    setAlliance(AllianceStationID.Blue1);
    connect(1);
    teleport(new Pose2d(3.0, 2.0, Rotation2d.kZero));
    setAxis(1, AXIS_LEFT_Y, -1.0);
    for (int i = 0; i < 50; i++) stepRobot();
    setAxis(1, AXIS_LEFT_Y, 0.0);
    for (int i = 0; i < 50; i++) stepRobot(); // 1 s of braking

    ChassisSpeeds speeds = speeds();
    assertTrue(Math.abs(speeds.vxMetersPerSecond) < 0.3, "vx at rest: " + speeds);
    assertTrue(Math.abs(speeds.vyMetersPerSecond) < 0.3, "vy at rest: " + speeds);
    assertTrue(Math.abs(speeds.omegaRadiansPerSecond) < 0.5, "omega at rest: " + speeds);
  }

  @Test
  void drivePercentScalesSettledForwardSpeed() {
    setAlliance(AllianceStationID.Blue1);
    connect(1);
    teleport(new Pose2d(2.0, 2.0, Rotation2d.kZero));
    setAxis(1, AXIS_LEFT_Y, -1.0);
    double full = meanVx(90);
    assertTrue(full > 1.0, "sanity: 100% moves, |vx|=" + full);

    DashboardControlsTestUtil.set("Drive Speed Limit (%)", 50.0);
    double half = meanVx(60);
    assertTrue(
        half > 0.25 * full && half < 0.75 * full,
        "50% scales translation: full=" + full + " half=" + half);
    setAxis(1, AXIS_LEFT_Y, 0.0);
    DashboardControlsTestUtil.set("Drive Speed Limit (%)", 100.0);
  }

  @Test
  void turnPercentScalesSettledRotation() {
    setAlliance(AllianceStationID.Blue1);
    connect(1);
    teleport(new Pose2d(3.0, 4.0, Rotation2d.kZero));
    setAxis(1, AXIS_RIGHT_X, 1.0);
    double full = meanOmega(70);
    assertTrue(full > 1.0, "sanity: 100% turns, |omega|=" + full);

    DashboardControlsTestUtil.set("Turn Speed Limit (%)", 50.0);
    double half = meanOmega(60);
    assertTrue(
        half > 0.25 * full && half < 0.75 * full,
        "50% scales rotation: full=" + full + " half=" + half);
    setAxis(1, AXIS_RIGHT_X, 0.0);
    DashboardControlsTestUtil.set("Turn Speed Limit (%)", 100.0);
  }

  @Test
  void headingResetBlueZeroRedHundredEightyPositionKept() {
    connect(1);

    // Works while disabled, blue -> 0 deg.
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    teleport(new Pose2d(3.5, 2.5, Rotation2d.fromDegrees(137)));
    setAlliance(AllianceStationID.Blue1);
    press(1, BUTTON_B);
    assertEquals(0.0, pose().getRotation().getDegrees(), 0.5, "blue reset to 0");
    assertEquals(3.5, pose().getX(), 0.05, "position kept");
    assertEquals(2.5, pose().getY(), 0.05, "position kept");
    release(1, BUTTON_B);

    // Red -> 180 deg. (wrap-safe compare: the +/-pi representation boundary sits exactly there)
    teleport(new Pose2d(3.5, 2.5, Rotation2d.fromDegrees(137)));
    setAlliance(AllianceStationID.Red1);
    press(1, BUTTON_B);
    assertTrue(
        Math.abs(pose().getRotation().minus(Rotation2d.kPi).getDegrees()) < 0.5,
        "red reset to 180, got " + pose().getRotation().getDegrees());
    assertEquals(3.5, pose().getX(), 0.05, "position kept on red");
    assertEquals(2.5, pose().getY(), 0.05, "position kept on red");
    release(1, BUTTON_B);

    // Unknown alliance -> 0 deg (blue fallback).
    teleport(new Pose2d(3.5, 2.5, Rotation2d.fromDegrees(137)));
    setAlliance(AllianceStationID.Unknown);
    press(1, BUTTON_B);
    assertEquals(0.0, pose().getRotation().getDegrees(), 0.5, "unknown resets to 0");
    release(1, BUTTON_B);

    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  // ---------------------------------------------------------------------------
  // Item 8: dual-controller routing through the real bindings
  // ---------------------------------------------------------------------------

  @Test
  void noControllersYieldsZeroDriveAndMinusOneActiveController() {
    // Both ports disconnected (BeforeEach state): axes read zero, nothing moves.
    for (int i = 0; i < 30; i++) stepRobot();
    ChassisSpeeds speeds = speeds();
    assertTrue(Math.abs(speeds.vxMetersPerSecond) < 0.3, "no controllers: vx ~ 0: " + speeds);
    assertTrue(Math.abs(speeds.vyMetersPerSecond) < 0.3, "no controllers: vy ~ 0");
    assertTrue(Math.abs(speeds.omegaRadiansPerSecond) < 0.5, "no controllers: omega ~ 0");

    double active =
        NetworkTableInstance.getDefault()
            .getEntry("/SmartDashboard/Active Controller")
            .getDouble(99);
    assertEquals(-1.0, active, EPS, "Active Controller key reads -1");
  }

  @Test
  void portOneWinsEvenWithZeroAxesAndPortZeroCannotDrive() {
    setAlliance(AllianceStationID.Blue1);
    teleport(new Pose2d(3.0, 2.0, Rotation2d.kZero));
    connect(0);
    connect(1); // demo pad connected but idle: it still owns control

    setAxis(0, AXIS_LEFT_Y, -1.0); // full port input must be ignored
    for (int i = 0; i < 40; i++) stepRobot();
    assertTrue(pose().getX() < 3.05 && pose().getX() > 2.95, "port 0 cannot drive: " + pose());

    assertEquals(
        1.0,
        NetworkTableInstance.getDefault()
            .getEntry("/SmartDashboard/Active Controller")
            .getDouble(99),
        EPS);

    setAxis(0, AXIS_LEFT_Y, 0.0);
    setAxis(1, AXIS_LEFT_Y, -1.0); // the demo pad's own stick works
    for (int i = 0; i < 40; i++) stepRobot();
    assertTrue(pose().getX() > 3.4, "port 1 drives: " + pose());
    setAxis(1, AXIS_LEFT_Y, 0.0);
  }

  @Test
  void portOneDisconnectHandsControlToPortZero() {
    setAlliance(AllianceStationID.Blue1);
    teleport(new Pose2d(3.0, 2.0, Rotation2d.kZero));
    connect(0);
    connect(1);
    disconnect(1);
    setAxis(0, AXIS_LEFT_Y, -1.0);
    for (int i = 0; i < 40; i++) stepRobot();
    assertTrue(pose().getX() > 3.4, "port 0 takes over after port 1 leaves: " + pose());
    assertEquals(
        0.0,
        NetworkTableInstance.getDefault()
            .getEntry("/SmartDashboard/Active Controller")
            .getDouble(99),
        EPS);
    setAxis(0, AXIS_LEFT_Y, 0.0);
  }

  @Test
  void controllerConnectionKeysTrackSimState() {
    connect(0);
    connect(1);
    settleToPublish();
    assertTrue(
        NetworkTableInstance.getDefault()
            .getEntry("/SmartDashboard/Controller 0 Connected")
            .getBoolean(false));
    assertTrue(
        NetworkTableInstance.getDefault()
            .getEntry("/SmartDashboard/Controller 1 Connected")
            .getBoolean(false));

    disconnect(1);
    settleToPublish();
    assertFalse(
        NetworkTableInstance.getDefault()
            .getEntry("/SmartDashboard/Controller 1 Connected")
            .getBoolean(true));
    assertTrue(
        NetworkTableInstance.getDefault()
            .getEntry("/SmartDashboard/Controller 0 Connected")
            .getBoolean(false));
  }

  @Test
  void portOneUnboundKeysTriggerNothing() {
    connect(1);
    // X (Manual), A (current-feed Shooting), Y (zeroing) exist on port 0 only.
    press(1, BUTTON_X);
    press(1, BUTTON_A);
    press(1, BUTTON_Y);
    DriverStationSim.setJoystickAxis(1, 3, 1.0); // RT (full-only reverse)
    stepRobot();
    for (int i = 0; i < 20; i++) stepRobot();
    assertFalse(
        Math.abs(
                Drum.mInstance.getVelocitySetpointRotPerSec()
                    - ManualConstants.ShooterRotpsTunable.getAsDouble())
            < EPS,
        "Manual must not start from port 1's X: drum="
            + Drum.mInstance.getVelocitySetpointRotPerSec());
    assertEquals(0.0, feederSetpoint(), EPS, "feeder untouched by port 1 full-only keys");
    assertEquals(0.0, feederAmps(), EPS, "no current request from port 1 full-only keys");
    release(1, BUTTON_X);
    release(1, BUTTON_A);
    release(1, BUTTON_Y);
    DriverStationSim.setJoystickAxis(1, 3, 0.0);
    stepRobot();
  }

  // ---------------------------------------------------------------------------
  // Item 6: pause + release / interruption through the real bindings
  // ---------------------------------------------------------------------------

  @Test
  void releaseDuringPauseDoesNotAutoResumeAndRepressWorks() {
    connect(1);
    press(1, BUTTON_RB);
    assertTrue(stepUntilFeeding(40), "Shooting(false) reached SHOOT via the RB binding");
    settleToPublish();
    assertTrue(feedAllowedKey(), "panel allows while feeding");

    // Lose readiness mid-volley and release the key while paused.
    DashboardControlsTestUtil.set(FLYWHEEL, 0.0);
    stepRobot();
    settle();
    assertEquals(0.0, feederSetpoint(), EPS, "feed stopped on the pause loop");
    settleToPublish();
    assertFalse(feedAllowedKey(), "panel blocks while paused");
    assertEquals(ShotReadiness.REASON_FLYWHEEL_ZERO, feedBlockedReasonKey());

    release(1, BUTTON_RB);
    DashboardControlsTestUtil.set(FLYWHEEL, 100.0);
    for (int i = 0; i < 25; i++) { // 0.5 s: far past the 0.15 s debounce
      stepRobot();
      assertEquals(0.0, feederSetpoint(), EPS, "no auto-resume after release, loop " + i);
    }
    settleToPublish();
    assertEquals("Idle", robotAction(), "action fell back to Idle after the heartbeat expired");
    assertFalse(feedAllowedKey(), "fallback preview: drum at stow, not allowed");
    assertEquals(ShotReadiness.REASON_DRUM_NOT_AT_SPEED, feedBlockedReasonKey());

    // A fresh press must work — no locked state.
    press(1, BUTTON_RB);
    assertTrue(stepUntilFeeding(40), "re-press starts a fresh volley");
    release(1, BUTTON_RB);
  }

  @Test
  void releaseDuringPauseDoesNotAutoResumeCurrentFeedMode() {
    connect(0); // A = Shooting(true) exists on port 0 only
    press(0, BUTTON_A);
    boolean currentFeeding = false;
    for (int i = 0; i < 40 && !currentFeeding; i++) {
      stepRobot();
      currentFeeding =
          Feeder.mInstance.getControlMode() == MotorIO.ControlMode.CURRENT && feederAmps() > 49.0;
    }
    assertTrue(currentFeeding, "current-feed mode reached SHOOT (50 A request)");

    DashboardControlsTestUtil.set(FLYWHEEL, 0.0);
    stepRobot();
    settle();
    assertEquals(0.0, feederAmps(), EPS, "current request withdrawn on the pause loop");
    assertEquals(0.0, feederSetpoint(), EPS);

    release(0, BUTTON_A);
    DashboardControlsTestUtil.set(FLYWHEEL, 100.0);
    for (int i = 0; i < 25; i++) {
      stepRobot();
      assertEquals(0.0, feederSetpoint(), EPS, "no auto-resume, loop " + i);
      assertEquals(0.0, feederAmps(), EPS, "no current request returns, loop " + i);
    }

    press(0, BUTTON_A);
    boolean resumed = false;
    for (int i = 0; i < 40 && !resumed; i++) {
      stepRobot();
      resumed = feederAmps() > 49.0;
    }
    assertTrue(resumed, "re-press restarts current-feed shooting");
    release(0, BUTTON_A);
  }

  @Test
  void releaseDuringPrepareDoesNotAutoResumeManual() {
    connect(0); // X = Manual exists on port 0 only
    press(0, BUTTON_X);
    assertTrue(stepUntilManualFeeding(40), "Manual reached SHOOT via the X binding");

    DashboardControlsTestUtil.set(FLYWHEEL, 0.0);
    stepRobot();
    settle();
    assertEquals(0.0, feederSetpoint(), EPS, "Manual dropped to PREPARE on the detection loop");

    release(0, BUTTON_X);
    DashboardControlsTestUtil.set(FLYWHEEL, 100.0);
    for (int i = 0; i < 25; i++) {
      stepRobot();
      assertEquals(0.0, feederSetpoint(), EPS, "Manual never auto-resumes, loop " + i);
    }

    press(0, BUTTON_X);
    assertTrue(stepUntilManualFeeding(40), "re-press restarts Manual");
    release(0, BUTTON_X);
  }

  @Test
  void conflictingCommandInterruptsPausedShootingAndCleanRestartFollows() {
    connect(1);
    press(1, BUTTON_RB);
    assertTrue(stepUntilFeeding(40), "reached SHOOT");

    DashboardControlsTestUtil.set(FLYWHEEL, 0.0);
    stepRobot();
    settle();
    assertEquals(0.0, feederSetpoint(), EPS, "paused");

    // LT starts IntakeCommand, whose requirements overlap Shooting's: legal interruption.
    setTrigger(1, 0.9);
    settle();
    assertEquals(
        90.0,
        IntakeRoller.mInstance.getVelocitySetpointRotPerSec(),
        EPS,
        "intake roller is running");
    assertEquals(0.0, feederSetpoint(), EPS, "Shooting was interrupted, feeder stopped");

    setTrigger(1, 0.0); // release LT
    DashboardControlsTestUtil.set(FLYWHEEL, 100.0);
    for (int i = 0; i < 25; i++) {
      stepRobot();
      assertEquals(0.0, feederSetpoint(), EPS, "interrupted command never comes back, loop " + i);
    }

    // The RB press that started the interrupted volley never rising-edges again while held, so
    // the re-press needs a release first — same held-button rule as the disable case.
    release(1, BUTTON_RB);
    press(1, BUTTON_RB); // fresh press must work after the interruption
    assertTrue(stepUntilFeeding(40), "clean restart after interruption");
    release(1, BUTTON_RB);
  }

  @Test
  void disableMidVolleyCancelsShootingAndHeldButtonNeedsRepress() {
    connect(1);
    press(1, BUTTON_RB);
    assertTrue(stepUntilFeeding(40), "reached SHOOT");

    DriverStationSim.setEnabled(false);
    stepRobot();
    settle();
    assertEquals(0.0, feederSetpoint(), EPS, "disable canceled the volley (end() stopped feed)");
    assertEquals(DrumConstants.StowVelocity, Drum.mInstance.getVelocitySetpointRotPerSec(), EPS);

    // Re-enable with the button still held: record actual behavior — no new rising edge.
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    for (int i = 0; i < 25; i++) {
      stepRobot();
      assertEquals(
          DrumConstants.StowVelocity,
          Drum.mInstance.getVelocitySetpointRotPerSec(),
          EPS,
          "held-through-disable button does not restart, loop " + i);
    }

    release(1, BUTTON_RB);
    press(1, BUTTON_RB);
    assertTrue(stepUntilFeeding(40), "release + re-press restarts after a disable cycle");
    release(1, BUTTON_RB);
  }

  @Test
  void pressWhileDisabledThenEnableStillHeldDoesNotStartIntake() {
    connect(1);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();

    setTrigger(1, 0.9); // LT pressed while disabled
    for (int i = 0; i < 10; i++) stepRobot();
    settle();
    assertEquals(0.0, IntakeRoller.mInstance.getVelocitySetpointRotPerSec(), EPS);

    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    for (int i = 0; i < 10; i++) {
      stepRobot();
      assertEquals(
          0.0,
          IntakeRoller.mInstance.getVelocitySetpointRotPerSec(),
          EPS,
          "held-through-disable trigger does not start intake, loop " + i);
    }

    setTrigger(1, 0.0);
    setTrigger(1, 0.9); // re-press while enabled works
    boolean intakeRunning = false;
    for (int i = 0; i < 10 && !intakeRunning; i++) {
      stepRobot();
      intakeRunning = IntakeRoller.mInstance.getVelocitySetpointRotPerSec() != 0.0;
    }
    assertTrue(intakeRunning, "re-press while enabled starts intake");
    setTrigger(1, 0.0);
  }

  @Test
  void reconnectWhileButtonHeldRestartsHeldCommand() {
    connect(1);
    press(1, BUTTON_RB);
    assertTrue(stepUntilFeeding(40), "reached SHOOT");

    disconnect(1);
    settle();
    assertEquals(0.0, feederSetpoint(), EPS, "disconnect interrupted the volley");

    // Reconnect WITHOUT releasing RB: the trigger rises again, so the binding re-fires. This is
    // the actual behavior — recorded here so a change to it is noticed.
    DriverStationSim.setJoystickButton(1, BUTTON_RB, true);
    connect(1);
    boolean resumed = false;
    for (int i = 0; i < 40 && !resumed; i++) {
      stepRobot();
      resumed =
          Math.abs(feederSetpoint() - ShootingConstants.FeederRotpsTunable.getAsDouble()) < EPS;
    }
    assertTrue(resumed, "button held across a reconnect restarts the held command");
    release(1, BUTTON_RB);
  }

  @Test
  void portOneTakeoverInterruptsPortZeroShooting() {
    connect(0);
    press(0, BUTTON_RB);
    assertTrue(stepUntilFeeding(40), "port 0 shooting");

    connect(1); // takeover: port 0's held command must be interrupted by priority loss
    settle();
    assertEquals(0.0, feederSetpoint(), EPS, "takeover stopped port 0's volley");
    assertEquals(DrumConstants.StowVelocity, Drum.mInstance.getVelocitySetpointRotPerSec(), EPS);

    // The panel cannot be pinned by the interrupted command: the FEEDING heartbeat must expire
    // (0.15 s) and a fresh publish must fire before "Idle" shows.
    for (int i = 0; i < 10; i++) stepRobot();
    settleToPublish();
    assertEquals("Idle", robotAction());
    release(0, BUTTON_RB);
  }

  @Test
  void fullDisconnectStopsIntakeAndDrive() {
    setAlliance(AllianceStationID.Blue1);
    teleport(new Pose2d(3.0, 2.0, Rotation2d.kZero));
    connect(1);
    setTrigger(1, 0.9); // LT: intake
    boolean intakeRunning = false;
    for (int i = 0; i < 10 && !intakeRunning; i++) {
      stepRobot();
      intakeRunning = IntakeRoller.mInstance.getVelocitySetpointRotPerSec() != 0.0;
    }
    assertTrue(intakeRunning, "intake running");

    setAxis(1, AXIS_LEFT_Y, -1.0); // and driving
    for (int i = 0; i < 30; i++) stepRobot();
    assertTrue(pose().getX() > 3.3, "was driving: " + pose());

    disconnect(1); // every controller gone
    settle();
    assertEquals(0.0, IntakeRoller.mInstance.getVelocitySetpointRotPerSec(), EPS);
    for (int i = 0; i < 50; i++) stepRobot();
    ChassisSpeeds speeds = speeds();
    assertTrue(Math.abs(speeds.vxMetersPerSecond) < 0.3, "drive input zeroed, at rest: " + speeds);
  }

  @Test
  void homingCommandStopsWhenItsControllerDisconnects() {
    connect(1);
    setPov(1, 270); // D-pad left: IntakeDeployZeroCommand (long one-shot homing)
    // Sample early: in sim the modeled stall current can satisfy the 0.15 s spike debounce and
    // finish the homing on its own, which is fine — the disconnect must stop it before that.
    double volts = 0;
    for (int i = 0; i < 3; i++) {
      stepRobot();
      volts = IntakeDeploy.mInstance.getVoltageSetpoint();
    }
    assertTrue(Math.abs(volts) > 0.5, "homing drives the deploy motor, volts=" + volts);

    disconnect(1);
    settle();
    assertEquals(
        0.0,
        IntakeDeploy.mInstance.getVoltageSetpoint(),
        EPS,
        "homing command ended on disconnect, motor stopped");
    setPov(1, -1);
  }

  @Test
  void momentaryEjectWritesReverseThenZeroesOnRelease() {
    connect(1);
    press(1, BUTTON_LB);
    settle();
    // Momentary reverse: no subsystem ownership, -3 V writes to feeder/indexer/roller.
    assertTrue(
        Feeder.mInstance.getVoltageSetpoint() < -1.0,
        "LB writes reverse volts, got " + Feeder.mInstance.getVoltageSetpoint());
    release(1, BUTTON_LB);
    settle();
    assertEquals(0.0, Feeder.mInstance.getVoltageSetpoint(), EPS, "release zeroes");
  }

  @Test
  void operatorPanelKeysPublishTypesAndValues() {
    connect(1);
    press(1, BUTTON_RB);
    assertTrue(stepUntilFeeding(40), "feeding");
    settleToPublish(); // guarantee the 10 Hz publish fired after entering SHOOT

    var nt = NetworkTableInstance.getDefault();
    assertTrue(nt.getEntry("/SmartDashboard/Robot Action").exists());
    assertEquals("Feeding", robotAction());
    assertTrue(nt.getEntry("/SmartDashboard/Feed Allowed").exists());
    assertTrue(feedAllowedKey());
    assertTrue(nt.getEntry("/SmartDashboard/Feed Blocked Reason").exists());
    assertEquals("", feedBlockedReasonKey());
    assertTrue(nt.getEntry("/SmartDashboard/Drum Target Speed (rot/s)").exists());
    assertEquals(
        DrumConstants.MaxShotVelocity,
        nt.getEntry("/SmartDashboard/Drum Target Speed (rot/s)").getDouble(-1),
        EPS);
    assertTrue(nt.getEntry("/SmartDashboard/Drum Actual Speed (rot/s)").exists());
    assertTrue(nt.getEntry("/SmartDashboard/Battery Voltage (V)").exists());
    assertTrue(nt.getEntry("/SmartDashboard/Battery Voltage (V)").getDouble(0) > 5.0);
    assertTrue(nt.getEntry("/SmartDashboard/Controller 1 Connected").getBoolean(false));
    release(1, BUTTON_RB);
  }
}
