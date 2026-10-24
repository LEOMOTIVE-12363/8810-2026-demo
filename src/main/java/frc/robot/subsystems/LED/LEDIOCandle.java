package frc.robot.subsystems.LED;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import java.util.Arrays;
import java.util.function.DoubleSupplier;

/**
 * CTRE CANdle driving the WS2811 strip from its VBat (12 V) output. Selected by {@link
 * LEDConstants#HARDWARE} = CANDLE; otherwise never constructed, so no CANdle device is created at
 * all (and vice versa: the PWM output is never opened in this mode).
 *
 * <p>Index mapping: the CANdle's own board LEDs sit at device indices 0-7 and the external strip
 * starts at index 8, so the upper layers keep their zero-based strip indexes and this driver adds
 * the offset. The onboard LEDs are NOT counted in the strip length and never touch {@link
 * LEDSection} — they show the logical ball-path thumbnail supplied through {@link
 * #writeOnboardBuffer(AddressableLEDBuffer)} (rendered by the shared pipeline at eight normalized u
 * positions, onboard 0→7 = intake→shooter; user decision 2026-10-24, refined in the second
 * acceptance round from physical-index sampling so reversed sections, split side panels and
 * decoration runs can never change the onboard logical direction). Until the render layer supplies
 * it, the thumbnail defaults to dark. Per the CANdle User's Guide the firmware compensates the
 * onboard LEDs' color order itself, so the external strip-type config (GRB for the WS2811 R/G swap)
 * never corrupts the onboard colors; the 5 V rail is enabled because a disabled rail turns the
 * onboard LEDs off entirely.
 *
 * <p>Color order: the incoming buffers hold logical RGB and the CANdle's strip-type config does the
 * physical channel ordering for the external strip (GRB when the WS2811 swap flag is set), so no
 * software swap happens here — exactly one conversion per pixel. Brightness is already applied by
 * the render layer, and the device-side brightness scalar is pinned to 1.0 in the config so a value
 * persisted in the CANdle's flash (e.g. from a Tuner X experiment) can never scale it a second
 * time.
 *
 * <p>Bounded sending with per-pixel confirmation: each {@code SolidColor} control is one one-shot
 * CAN frame painting one contiguous same-color run (verified against the Phoenix 6 26.3.0 API: the
 * request carries an inclusive start/end index plus one color). The driver tracks, per logical
 * pixel (onboard + external), the color the device is believed to show; only pixels whose paint
 * returned OK update that belief. Therefore: (1) a settled display needs no traffic, even when a
 * frame was completed across several partial refreshes; (2) a failed paint leaves its pixels
 * pending, retried on later eligible ticks within the same frame budget — and because a failure
 * makes the whole device state suspect, one full bounded repaint follows recovery; (3) adjacent
 * same-color dirty pixels coalesce into single frames; (4) at most {@code maxFramesPerUpdate}
 * frames go out per update tick, spaced at least {@code minUpdateIntervalSec} apart, with a
 * rotating cursor so a continuously-changing frame cannot starve the tail; and (5) every tick plans
 * from the latest target colors — stale frames are never queued. Worst case = maxFrames / interval
 * frames per second and ceil((onboard + strip) / maxFrames) ticks per full refresh (see {@link
 * LEDConstants#CANDLE_MIN_UPDATE_INTERVAL_SEC}).
 *
 * <p>Recovery (second acceptance round, 2026-10-24): initialization is a tracked state, not
 * fire-and-forget — if the device config (power rails, strip type, brightness scalar, animation
 * cleanup) fails at construction, it is retried at most once per {@link
 * LEDConstants#CANDLE_INIT_RETRY_INTERVAL_SEC} and only while the device answers its status signal
 * (a cached, non-blocking read), so an absent device never blocks the loop with config timeouts;
 * the config apply itself carries an explicit timeout. A detected device reset (Phoenix 6 {@code
 * hasResetOccurred}, also non-blocking) re-applies the config — a reset reverts it to flash
 * defaults — at most once per refresh interval. And because a reset can also happen without any
 * signal reaching this driver, every {@link LEDConstants#CANDLE_REFRESH_INTERVAL_SEC} a responsive
 * device has its configuration re-applied and the per-pixel confirmation invalidated as a bounded
 * keepalive, so both flash-default configuration and stale colors are recovered: static displays
 * are refreshed at a low bounded rate, not strictly zero traffic. After any successful
 * (re-)initialization the confirmation cache is invalidated and the latest target repaints under
 * the same budget.
 *
 * <p>Failure isolation: every device call returns a status code instead of throwing; faults are
 * reported on first occurrence, at most every 10 s while persisting, plus one recovery note. A dead
 * or missing CANdle never crashes the loop and never silently falls back to another driver.
 */
public class LEDIOCandle implements LEDIO {
  /** The CANdle's onboard LEDs occupy device indices 0-7; external pixels start at 8. */
  static final int ONBOARD_LED_COUNT = 8;

  /** Phoenix 6 documents external-strip indices 8-399 (inclusive), i.e. 392 external pixels max. */
  static final int MAX_EXTERNAL_PIXELS = 392;

  private static final double FAULT_LOG_INTERVAL_SEC = 10.0;

  /**
   * The device operations the driver needs; a real CANdle in REAL, the none() stub in SIM. The two
   * default probes let minimal fakes (and the acceptance-test buses) stay three-method
   * implementations.
   */
  interface Transport {
    StatusCode configureStrip(StripTypeValue stripType);

    StatusCode clearAnimations();

    StatusCode paint(int startInclusive, int endInclusive, int red, int green, int blue);

    /**
     * Non-blocking hint that the device is answering (cached status signal) — gates config retries
     * so they never block the loop on an absent device.
     */
    default boolean isDeviceResponsive() {
      return true;
    }

    /** Non-blocking client-side reset flag (Phoenix 6 {@code hasResetOccurred}). */
    default boolean hasResetOccurred() {
      return false;
    }
  }

  /** Wraps a CANdle handle — a real device, or the no-op {@link CANdle#none()} stub for sim. */
  static final class DeviceTransport implements Transport {
    private final CANdle candle;

    DeviceTransport(CANdle candle) {
      this.candle = candle;
    }

    @Override
    public StatusCode configureStrip(StripTypeValue stripType) {
      // Explicit timeout (seconds) keeps the worst-case block bounded; only called while the
      // device answers its status signal, so the timeout is rarely ever waited out.
      return candle.getConfigurator().apply(buildStripConfig(stripType), 0.1);
    }

    @Override
    public StatusCode clearAnimations() {
      return candle.clearAllAnimations();
    }

    @Override
    public StatusCode paint(int startInclusive, int endInclusive, int red, int green, int blue) {
      return candle.setControl(
          new SolidColor(startInclusive, endInclusive).withColor(new RGBWColor(red, green, blue)));
    }

    @Override
    public boolean isDeviceResponsive() {
      // Cached, non-blocking: OK means a status frame arrived recently, i.e. the device is present.
      return candle.getVersion().getStatus().isOK();
    }

    @Override
    public boolean hasResetOccurred() {
      return candle.hasResetOccurred();
    }
  }

  /**
   * The one-time device config: strip channel order, brightness pinned to 1.0 (the render layer
   * already applies it — never twice), the 12 V strip powered from VBat, and the 5 V rail enabled
   * because it feeds the onboard LEDs (a disabled rail turns them off; nothing is attached to it).
   */
  static CANdleConfiguration buildStripConfig(StripTypeValue stripType) {
    CANdleConfiguration config = new CANdleConfiguration();
    config.LED.StripType = stripType;
    config.LED.BrightnessScalar = 1.0; // brightness is applied once, in the render layer
    config.CANdleFeatures.VBatOutputMode = VBatOutputModeValue.On; // 12 V strip supply
    config.CANdleFeatures.Enable5VRail =
        Enable5VRailValue.Enabled; // powers the onboard LEDs; no external 5 V strip attached
    return config;
  }

  private final Transport transport;
  private final int lengthPixels;
  private final boolean swapRedGreen;
  private final double minUpdateIntervalSec;
  private final int maxFramesPerUpdate;
  private final DoubleSupplier clockSec;

  /** Colors the device is believed to show, by logical pixel (onboard 0-7, then external). */
  private final int[] confirmedPacked;

  private final int[] targetPacked;
  private final int[] runStart;
  private final int[] runEnd;
  private final int[] runPacked;

  /** Latest onboard thumbnail colors from the render layer (packed), default dark. */
  private final int[] onboardTargetPacked = new int[ONBOARD_LED_COUNT];

  private double lastSendSec = Double.NEGATIVE_INFINITY;
  private double lastFaultLogSec = Double.NEGATIVE_INFINITY;
  private double lastInitAttemptSec;
  private double lastResetInitSec;
  private double lastForcedRefreshSec;
  private int cursorLogical;
  private boolean initSucceeded;
  private boolean faulted;
  private boolean warnedTruncated;

  /** Test/injection constructor; see {@link #forReal} / {@link #forSim} for the real paths. */
  LEDIOCandle(
      Transport transport,
      int lengthPixels,
      boolean swapRedGreen,
      double minUpdateIntervalSec,
      int maxFramesPerUpdate,
      DoubleSupplier clockSec) {
    this.transport = transport;
    this.lengthPixels = Math.max(lengthPixels, 1);
    this.swapRedGreen = swapRedGreen;
    this.minUpdateIntervalSec = minUpdateIntervalSec;
    this.maxFramesPerUpdate = Math.max(maxFramesPerUpdate, 1);
    this.clockSec = clockSec;
    int logicalPixels = ONBOARD_LED_COUNT + Math.min(this.lengthPixels, MAX_EXTERNAL_PIXELS);
    confirmedPacked = new int[logicalPixels];
    targetPacked = new int[logicalPixels];
    runStart = new int[logicalPixels];
    runEnd = new int[logicalPixels];
    runPacked = new int[logicalPixels];
    // -1 never matches a packed color (all are >= 0), so unconfirmed pixels are always pending.
    Arrays.fill(confirmedPacked, -1);
    double now = clockSec.getAsDouble();
    lastInitAttemptSec = now;
    lastResetInitSec = now;
    lastForcedRefreshSec = now;
    initializeDevice(now);
  }

  /** REAL: a real CANdle on the configured CAN id/bus (see {@link LEDConstants}). */
  public static LEDIOCandle forReal(boolean swapRedGreen) {
    return new LEDIOCandle(
        new DeviceTransport(
            new CANdle(LEDConstants.CANDLE_CAN_ID, new CANBus(LEDConstants.CANDLE_CAN_BUS))),
        LEDConstants.LENGTH_PIXELS,
        swapRedGreen,
        LEDConstants.CANDLE_MIN_UPDATE_INTERVAL_SEC,
        LEDConstants.CANDLE_MAX_FRAMES_PER_UPDATE,
        Timer::getFPGATimestamp);
  }

  /**
   * SIM: the identical bounded-send logic against the CANdle.none() stub — controls and configs are
   * no-ops that return OK, so the sim runs fully offline without creating a CAN device.
   */
  public static LEDIOCandle forSim(boolean swapRedGreen) {
    return new LEDIOCandle(
        new DeviceTransport(CANdle.none()),
        LEDConstants.LENGTH_PIXELS,
        swapRedGreen,
        LEDConstants.CANDLE_MIN_UPDATE_INTERVAL_SEC,
        LEDConstants.CANDLE_MAX_FRAMES_PER_UPDATE,
        Timer::getFPGATimestamp);
  }

  /** Records the latest onboard thumbnail colors; no CAN traffic by itself. */
  @Override
  public void writeOnboardBuffer(AddressableLEDBuffer buffer) {
    for (int s = 0; s < ONBOARD_LED_COUNT; s++) {
      onboardTargetPacked[s] =
          s < buffer.getLength()
              ? (buffer.getRed(s) << 16) | (buffer.getGreen(s) << 8) | buffer.getBlue(s)
              : 0;
    }
  }

  /**
   * One-time device setup: config, animation-slot cleanup, onboard LEDs dark. Failure leaves the
   * driver uninitialized — the maintenance pass retries at a bounded rate. Success invalidates the
   * confirmation cache (config changes what the device would show) so the latest target repaints.
   */
  private StatusCode configureHardware(double now) {
    lastInitAttemptSec = now;
    StripTypeValue stripType = swapRedGreen ? StripTypeValue.GRB : StripTypeValue.RGB;
    return transport.configureStrip(stripType);
  }

  private void initializeDevice(double now) {
    if (lengthPixels > MAX_EXTERNAL_PIXELS && !warnedTruncated) {
      warnedTruncated = true;
      System.err.println(
          "[LEDIOCandle] LENGTH_PIXELS ("
              + lengthPixels
              + ") exceeds the CANdle external maximum ("
              + MAX_EXTERNAL_PIXELS
              + "); external pixels beyond it are not sent");
    }
    StatusCode configStatus = configureHardware(now);
    StatusCode clearStatus = transport.clearAnimations();
    StatusCode onboardStatus =
        transport.paint(0, ONBOARD_LED_COUNT - 1, 0, 0, 0); // deterministic dark until first frame
    boolean ok =
        configStatus != null
            && configStatus.isOK()
            && clearStatus != null
            && clearStatus.isOK()
            && onboardStatus != null
            && onboardStatus.isOK();
    initSucceeded = ok;
    if (ok) {
      // Config (re)applied: what the device shows may have changed underneath — repaint everything.
      Arrays.fill(confirmedPacked, -1);
    } else {
      logFaultRateLimited(
          now,
          "initialization failed (config "
              + configStatus
              + ", clear "
              + clearStatus
              + ", onboard "
              + onboardStatus
              + "); will retry while the device answers");
    }
  }

  @Override
  public void writeBuffer(AddressableLEDBuffer buffer) {
    int external = Math.min(buffer.getLength(), Math.min(lengthPixels, MAX_EXTERNAL_PIXELS));
    double now = clockSec.getAsDouble();
    maintenance(now);

    int logicalPixels = ONBOARD_LED_COUNT + external;
    for (int s = 0; s < ONBOARD_LED_COUNT; s++) {
      targetPacked[s] = onboardTargetPacked[s];
    }
    boolean dirty = false;
    for (int i = 0; i < external; i++) {
      int packed = (buffer.getRed(i) << 16) | (buffer.getGreen(i) << 8) | buffer.getBlue(i);
      targetPacked[ONBOARD_LED_COUNT + i] = packed;
      dirty |= packed != confirmedPacked[ONBOARD_LED_COUNT + i];
    }
    for (int s = 0; s < ONBOARD_LED_COUNT && !dirty; s++) {
      dirty |= targetPacked[s] != confirmedPacked[s];
    }
    if (!dirty) {
      return; // firmware already shows exactly these colors; no CAN traffic this tick
    }
    if (now - lastSendSec < minUpdateIntervalSec) {
      return; // throttled; the dirty state survives until an eligible tick
    }
    lastSendSec = now;
    sendRuns(now, logicalPixels);
  }

  /**
   * Bounded-recovery pass, run every loop before planning: retry a failed initialization (only
   * while the device answers, at most once per init-retry interval), re-apply the config after a
   * detected device reset, and periodically re-apply config plus invalidate confirmations so an
   * undetected reset restores both the device configuration and the latest target.
   */
  private void maintenance(double now) {
    if (!initSucceeded) {
      if (now - lastInitAttemptSec >= LEDConstants.CANDLE_INIT_RETRY_INTERVAL_SEC
          && transport.isDeviceResponsive()) {
        initializeDevice(now);
      }
    } else if (now - lastResetInitSec >= LEDConstants.CANDLE_REFRESH_INTERVAL_SEC
        && transport.hasResetOccurred()) {
      // Interval check first: the reset flag must not be consumed while the re-init is still
      // rate-limited, or a reset inside the window would be silently swallowed.
      lastResetInitSec = now;
      initializeDevice(now);
    } else if (now - lastForcedRefreshSec >= LEDConstants.CANDLE_REFRESH_INTERVAL_SEC) {
      lastForcedRefreshSec = now;
      if (transport.isDeviceResponsive()) {
        StatusCode configStatus = configureHardware(now);
        initSucceeded = configStatus != null && configStatus.isOK();
        if (!initSucceeded) {
          logFaultRateLimited(
              now,
              "configuration refresh failed (" + configStatus + "); will retry while responsive");
        }
      }
      Arrays.fill(confirmedPacked, -1); // bounded keepalive: repaint latest target under budget
    }
  }

  /**
   * Plans same-color runs over the dirty pixels (target != confirmed) and paints up to {@code
   * maxFramesPerUpdate} of them, starting at the rotating cursor so continuously-changing frames
   * cannot starve the tail. Only OK paints update {@link #confirmedPacked}; a failed paint leaves
   * its pixels pending. Any failure also invalidates the whole confirmation (the device state is
   * suspect, e.g. after a reset), which triggers one bounded full repaint on recovery.
   */
  private void sendRuns(double now, int logicalPixels) {
    int runCount = 0;
    for (int pos = 0; pos < logicalPixels; pos++) {
      int packed = targetPacked[pos];
      if (packed == confirmedPacked[pos]) {
        continue; // already showing this color; skip
      }
      // Never merge across the onboard/external boundary: their device offsets differ.
      boolean canExtend =
          runCount > 0
              && runEnd[runCount - 1] == pos - 1
              && runPacked[runCount - 1] == packed
              && !(pos == ONBOARD_LED_COUNT);
      if (canExtend) {
        runEnd[runCount - 1] = pos;
      } else {
        runStart[runCount] = pos;
        runEnd[runCount] = pos;
        runPacked[runCount] = packed;
        runCount++;
      }
    }
    if (runCount == 0) {
      return;
    }

    // Rotate the plan so the run at/after the cursor goes first.
    int startRun = runCount - 1;
    for (int r = 0; r < runCount; r++) {
      if (runEnd[r] >= cursorLogical) {
        startRun = r;
        break;
      }
    }

    int sent = 0;
    int lastRunSent = -1;
    boolean anyFailure = false;
    StatusCode firstFailure = null;
    for (int k = 0; k < runCount && sent < maxFramesPerUpdate; k++) {
      // Logical index space == device index space (onboard 0-7, external logical 8+i = device 8+i).
      int run = (startRun + k) % runCount;
      int packed = runPacked[run];
      StatusCode status =
          transport.paint(
              runStart[run],
              runEnd[run],
              (packed >> 16) & 0xFF,
              (packed >> 8) & 0xFF,
              packed & 0xFF);
      sent++;
      lastRunSent = run;
      if (status != null && status.isOK()) {
        Arrays.fill(confirmedPacked, runStart[run], runEnd[run] + 1, packed);
      } else {
        anyFailure = true;
        if (firstFailure == null) {
          firstFailure = status;
        }
      }
    }
    if (lastRunSent >= 0) {
      int after = runEnd[lastRunSent] + 1;
      cursorLogical = after >= logicalPixels ? 0 : after;
    }
    if (anyFailure) {
      // The device's displayed state is suspect (it may have reset or dropped frames): forget
      // every confirmation so recovery triggers one bounded full repaint from the latest targets.
      Arrays.fill(confirmedPacked, -1);
    }
    reportStatus(anyFailure ? firstFailure : StatusCode.OK, now);
  }

  /** Rate-limited fault reporting: first occurrence, every 10 s while stuck, one recovery note. */
  private void reportStatus(StatusCode status, double now) {
    if (status != null && status.isOK()) {
      if (faulted) {
        faulted = false;
        System.err.println("[LEDIOCandle] CAN communication recovered");
      }
      return;
    }
    logFaultRateLimited(
        now,
        "CANdle update failed ("
            + status
            + "); LED output may be absent — check CAN ID "
            + LEDConstants.CANDLE_CAN_ID
            + " on bus \""
            + LEDConstants.CANDLE_CAN_BUS
            + "\"");
  }

  private void logFaultRateLimited(double now, String message) {
    boolean firstOrDue = !faulted || now - lastFaultLogSec >= FAULT_LOG_INTERVAL_SEC;
    if (firstOrDue) {
      lastFaultLogSec = now;
      System.err.println("[LEDIOCandle] " + message);
    }
    faulted = true;
  }
}
