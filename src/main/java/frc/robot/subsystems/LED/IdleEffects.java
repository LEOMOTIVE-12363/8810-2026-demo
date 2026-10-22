package frc.robot.subsystems.LED;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;
import java.util.function.Function;

/**
 * Pure math for the idle effects: theme gradient × temporal brightness envelope. Everything is a
 * function of the logical coordinate u (0 = intake … 1 = shooter) and an injected time in seconds,
 * so results are deterministic under log replay and unit-testable; no NT or hardware state is
 * touched here. All periods, widths and floors are preview proposals centralized in {@link
 * LEDConstants} for real-strip tuning.
 */
public final class IdleEffects {
  private IdleEffects() {}

  /**
   * Color at logical u/time for the effect. Only {@link IdleEffect#COLOR_STEP} rearranges colors
   * (wide blocks rotating through the theme's gradient with a short RGB crossfade between steps, so
   * there is no black/white strobing); every other effect shows the plain theme gradient.
   */
  public static Color colorAt(
      IdleEffect effect, Function<Double, Color> themeColor, double u, double timeSec) {
    if (effect != IdleEffect.COLOR_STEP) {
      return themeColor.apply(u);
    }
    int blocks = LEDConstants.IDLE_COLOR_STEP_BLOCK_COUNT;
    int block = MathUtil.clamp((int) Math.floor(u * blocks), 0, blocks - 1);
    double stepPosition = timeSec / LEDConstants.IDLE_COLOR_STEP_PERIOD_SEC;
    long step = (long) Math.floor(stepPosition);
    Color current = blockColor(themeColor, Math.floorMod(block - step, blocks), blocks);
    double secondsIntoStep = (stepPosition - step) * LEDConstants.IDLE_COLOR_STEP_PERIOD_SEC;
    double blend = secondsIntoStep / LEDConstants.IDLE_COLOR_STEP_BLEND_SEC;
    if (blend >= 1.0) {
      return current;
    }
    Color previous = blockColor(themeColor, Math.floorMod(block - (step - 1), blocks), blocks);
    return lerp(previous, current, blend);
  }

  /**
   * Brightness envelope in [0, 1] multiplying the theme color. STATIC and COLOR_STEP are steady at
   * full brightness; BREATHE moves the whole strip in phase, SLOW_FLOW is a wide traveling wave,
   * CHASE is one soft-edge band traveling the path.
   */
  public static double envelope(IdleEffect effect, double u, double timeSec) {
    return switch (effect) {
      case STATIC, COLOR_STEP -> 1.0;
      case BREATHE -> {
        double wave =
            0.5 + 0.5 * Math.sin(2.0 * Math.PI * timeSec / LEDConstants.IDLE_BREATHE_PERIOD_SEC);
        yield LEDConstants.IDLE_MIN_BRIGHTNESS + (1.0 - LEDConstants.IDLE_MIN_BRIGHTNESS) * wave;
      }
      case SLOW_FLOW -> {
        double wave =
            0.5
                + 0.5
                    * Math.sin(
                        2.0 * Math.PI * (u - timeSec / LEDConstants.IDLE_SLOW_FLOW_PERIOD_SEC));
        yield LEDConstants.IDLE_MIN_BRIGHTNESS + (1.0 - LEDConstants.IDLE_MIN_BRIGHTNESS) * wave;
      }
      case CHASE -> {
        double center = positiveModulo(timeSec / LEDConstants.IDLE_CHASE_PERIOD_SEC, 1.0);
        double halfWidth = LEDConstants.IDLE_CHASE_BAND_WIDTH_PATH / 2.0;
        double distance = Math.abs(u - center);
        distance = Math.min(distance, 1.0 - distance); // the band wraps around the path ends
        yield LEDConstants.IDLE_CHASE_FLOOR
            + (1.0 - LEDConstants.IDLE_CHASE_FLOOR) * sinSquaredBand(distance, halfWidth);
      }
    };
  }

  /**
   * sin²-shaped soft band profile: 1 at the center, falling smoothly to 0 at {@code halfWidth} with
   * zero slope at both ends, so band edges never pop.
   */
  static double sinSquaredBand(double distance, double halfWidth) {
    double x = MathUtil.clamp(1.0 - distance / halfWidth, 0.0, 1.0);
    double s = Math.sin(Math.PI / 2.0 * x);
    return s * s;
  }

  /** Floored modulo that also behaves for negative values (a reversing phase runs below zero). */
  static double positiveModulo(double value, double modulus) {
    double result = value % modulus;
    return result < 0.0 ? result + modulus : result;
  }

  /** Per-channel RGB lerp; never travels around the HSV wheel. */
  static Color lerp(Color a, Color b, double t) {
    return new Color(
        MathUtil.interpolate(a.red, b.red, t),
        MathUtil.interpolate(a.green, b.green, t),
        MathUtil.interpolate(a.blue, b.blue, t));
  }

  /** Scales a color's brightness by a 0..1 factor, per RGB channel. */
  static Color scale(Color color, double factor) {
    return new Color(color.red * factor, color.green * factor, color.blue * factor);
  }

  /** Converts a 0..1 channel value to an 8-bit component, clamped and rounded. */
  static int to255(double channel) {
    return (int) Math.round(MathUtil.clamp(channel, 0.0, 1.0) * 255.0);
  }

  /** Gradient color at the center of logical block {@code index} out of {@code blocks}. */
  private static Color blockColor(Function<Double, Color> themeColor, int index, int blocks) {
    return themeColor.apply((index + 0.5) / blocks);
  }
}
