package frc.robot.subsystems.LED;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.util.Color;
import java.util.function.Function;
import org.junit.jupiter.api.Test;

/**
 * Idle-effect math is the operator-visible base layer of the Day of the Dead themes. It is a pure
 * function of (effect, u, t), so the approved motion spec — periods, brightness floors, block
 * rotation, crossfade continuity — is pinned here directly instead of through the subsystem.
 */
class IdleEffectsTest {
  private static final double EPS = 1e-9;

  /** Distinct color per position (red ramps up, green ramps down) so block rotation is visible. */
  private static final Function<Double, Color> TEST_THEME = u -> new Color(u, 1.0 - u, 0.5);

  private static void assertColorEquals(Color expected, Color actual, double eps, String where) {
    assertEquals(expected.red, actual.red, eps, where + " red");
    assertEquals(expected.green, actual.green, eps, where + " green");
    assertEquals(expected.blue, actual.blue, eps, where + " blue");
  }

  private static double channel(Color c, int ch) {
    return switch (ch) {
      case 0 -> c.red;
      case 1 -> c.green;
      default -> c.blue;
    };
  }

  private static double blockCenter(int k) {
    return (k + 0.5) / LEDConstants.IDLE_COLOR_STEP_BLOCK_COUNT;
  }

  private static boolean colorEquals(Color a, Color b) {
    return Math.abs(a.red - b.red) < EPS
        && Math.abs(a.green - b.green) < EPS
        && Math.abs(a.blue - b.blue) < EPS;
  }

  private static double stepMid(int step) {
    return (step + 0.5) * LEDConstants.IDLE_COLOR_STEP_PERIOD_SEC;
  }

  @Test
  void nonColorStepEffectsRenderTheRawThemeColor() {
    // Brightness motion lives in envelope(); colorAt stays the pure theme gradient.
    for (IdleEffect effect :
        new IdleEffect[] {
          IdleEffect.STATIC, IdleEffect.BREATHE, IdleEffect.SLOW_FLOW, IdleEffect.CHASE
        }) {
      for (double u : new double[] {0.0, 0.31, 0.77, 1.0}) {
        assertColorEquals(
            TEST_THEME.apply(u),
            IdleEffects.colorAt(effect, TEST_THEME, u, 7.7),
            EPS,
            effect + " @ u=" + u);
      }
    }
  }

  @Test
  void staticAndColorStepEnvelopesAreOneAndTimeInvariant() {
    for (double u : new double[] {0.0, 0.42, 1.0}) {
      for (double t : new double[] {0.0, 3.3, 91.7}) {
        assertEquals(1.0, IdleEffects.envelope(IdleEffect.STATIC, u, t), EPS);
        assertEquals(1.0, IdleEffects.envelope(IdleEffect.COLOR_STEP, u, t), EPS);
      }
    }
  }

  @Test
  void breatheStaysWithinItsRangeAndRepeatsEveryPeriod() {
    double period = LEDConstants.IDLE_BREATHE_PERIOD_SEC;
    double min = Double.POSITIVE_INFINITY;
    double max = Double.NEGATIVE_INFINITY;
    for (int i = 0; i <= 480; i++) {
      double t = i * (period / 480.0);
      // The whole strip breathes in phase: no spatial variation.
      assertEquals(
          IdleEffects.envelope(IdleEffect.BREATHE, 0.13, t),
          IdleEffects.envelope(IdleEffect.BREATHE, 0.87, t),
          EPS,
          "breathe must be uniform across the strip");
      for (double u : new double[] {0.0, 0.5, 1.0}) {
        double env = IdleEffects.envelope(IdleEffect.BREATHE, u, t);
        assertTrue(
            env >= LEDConstants.IDLE_MIN_BRIGHTNESS - EPS, "breathe below its floor at t=" + t);
        assertTrue(env <= 1.0 + EPS, "breathe above full brightness at t=" + t);
        min = Math.min(min, env);
        max = Math.max(max, env);
        assertEquals(
            env,
            IdleEffects.envelope(IdleEffect.BREATHE, u, t + period),
            1e-9,
            "breathe must repeat every period");
      }
    }
    // Sinusoidal between the extremes: it must actually reach both ends of the range.
    assertTrue(min < LEDConstants.IDLE_MIN_BRIGHTNESS + 0.01, "breathe never reaches its floor");
    assertTrue(max > 0.99, "breathe never reaches full brightness");
  }

  @Test
  void slowFlowIsATravelingWaveWithinItsRange() {
    double period = LEDConstants.IDLE_SLOW_FLOW_PERIOD_SEC;
    for (double u : new double[] {0.0, 0.25, 0.5, 0.75, 1.0}) {
      for (double t = 0.0; t <= period; t += 0.05) {
        double env = IdleEffects.envelope(IdleEffect.SLOW_FLOW, u, t);
        assertTrue(env >= LEDConstants.IDLE_MIN_BRIGHTNESS - EPS, "slow-flow below its floor");
        assertTrue(env <= 1.0 + EPS, "slow-flow above full brightness");
      }
    }
  }

  @Test
  void slowFlowWaveTranslatesOnePeriodFractionPerSecond() {
    // A wave crossing the path in IDLE_SLOW_FLOW_PERIOD_SEC seconds moves 1/period of the path per
    // second, so shifting u by that amount and t by one second lands on the same wave phase.
    double shiftU = 1.0 / LEDConstants.IDLE_SLOW_FLOW_PERIOD_SEC;
    for (double u : new double[] {0.0, 0.125, 0.5, 0.75}) {
      for (double t : new double[] {0.4, 2.0, 5.5}) {
        assertEquals(
            IdleEffects.envelope(IdleEffect.SLOW_FLOW, u, t),
            IdleEffects.envelope(IdleEffect.SLOW_FLOW, u + shiftU, t + 1.0),
            1e-9,
            "translated wave at u=" + u + " t=" + t);
      }
    }
  }

  @Test
  void chaseKeepsTheFloorEverywhereAndReachesFullBrightnessOnTheBand() {
    boolean foundPeak = false;
    for (double t = 0.0; t <= LEDConstants.IDLE_CHASE_PERIOD_SEC; t += 0.1) {
      for (double u = 0.0; u <= 1.0 + EPS; u += 0.002) {
        double env = IdleEffects.envelope(IdleEffect.CHASE, u, t);
        assertTrue(env >= LEDConstants.IDLE_CHASE_FLOOR - EPS, "chase below its floor at u=" + u);
        foundPeak |= env > 0.95;
      }
    }
    assertTrue(foundPeak, "the soft band must reach full brightness near its center at some t");
  }

  @Test
  void chaseRepeatsEveryPeriod() {
    double period = LEDConstants.IDLE_CHASE_PERIOD_SEC;
    for (double u : new double[] {0.2, 0.5, 0.8}) {
      for (double t : new double[] {0.3, 1.1, 2.9}) {
        assertEquals(
            IdleEffects.envelope(IdleEffect.CHASE, u, t),
            IdleEffects.envelope(IdleEffect.CHASE, u, t + period),
            1e-9,
            "chase must repeat every period");
      }
    }
  }

  @Test
  void colorStepRotatesTheThemeOrderExactlyOneBlockPerStep() {
    // Mid-step sampling avoids the crossfade window. The color order must rotate by exactly one
    // block per step, in one consistent direction: block k at step s equals block k±1 at step s-1
    // (the spec fixes the rotation amount; the direction is a look-and-feel choice).
    int blocks = LEDConstants.IDLE_COLOR_STEP_BLOCK_COUNT;
    Boolean rotatesForward =
        null; // true: block k at step s shows what block k+1 showed at step s-1
    for (int step = 1; step <= 3; step++) {
      for (int k = 0; k < blocks; k++) {
        Color current =
            IdleEffects.colorAt(IdleEffect.COLOR_STEP, TEST_THEME, blockCenter(k), stepMid(step));
        Color fromNext =
            IdleEffects.colorAt(
                IdleEffect.COLOR_STEP,
                TEST_THEME,
                blockCenter((k + 1) % blocks),
                stepMid(step - 1));
        Color fromPrevious =
            IdleEffects.colorAt(
                IdleEffect.COLOR_STEP,
                TEST_THEME,
                blockCenter((k + blocks - 1) % blocks),
                stepMid(step - 1));
        boolean matchesNext = colorEquals(current, fromNext);
        boolean matchesPrevious = colorEquals(current, fromPrevious);
        assertTrue(
            matchesNext != matchesPrevious,
            "rotation must move the order by exactly one block (step "
                + step
                + ", block "
                + k
                + ")");
        if (rotatesForward == null) {
          rotatesForward = matchesNext;
        } else {
          assertEquals(
              rotatesForward, matchesNext, "rotation direction must be consistent everywhere");
        }
      }
    }
  }

  @Test
  void colorStepCrossfadesBetweenStepsWithoutJumpsOrStrobes() {
    double u = blockCenter(0);
    double period = LEDConstants.IDLE_COLOR_STEP_PERIOD_SEC;
    double blend = LEDConstants.IDLE_COLOR_STEP_BLEND_SEC;

    // Sampled window is wide enough to contain the blend however it is aligned to the boundary.
    double t0 = period - 0.15;
    double t1 = period + blend + 0.17;
    Color before = IdleEffects.colorAt(IdleEffect.COLOR_STEP, TEST_THEME, u, t0);
    Color after = IdleEffects.colorAt(IdleEffect.COLOR_STEP, TEST_THEME, u, t1);
    double maxGap =
        Math.max(
            Math.abs(after.red - before.red),
            Math.max(Math.abs(after.green - before.green), Math.abs(after.blue - before.blue)));
    assertTrue(maxGap > 0.1, "the rotation must actually change this block's color");

    double sampleDt = 0.01;
    Color prev = before;
    for (double t = t0 + sampleDt; t <= t1 + EPS; t += sampleDt) {
      Color now = IdleEffects.colorAt(IdleEffect.COLOR_STEP, TEST_THEME, u, t);
      for (int ch = 0; ch < 3; ch++) {
        double old = channel(before, ch);
        double fresh = channel(after, ch);
        double current = channel(now, ch);
        double previous = channel(prev, ch);
        // A crossfade is a convex mix of the two step colors: it can never detour to black/white.
        assertTrue(
            current >= Math.min(old, fresh) - 0.02 && current <= Math.max(old, fresh) + 0.02,
            "channel " + ch + " left the old-new segment (strobe?) at t=" + t);
        // Over a >= blend-long window a sampleDt step moves only a small fraction of the total
        // gap; an instantaneous jump between steps would exceed this bound.
        assertTrue(
            Math.abs(current - previous) <= maxGap * (sampleDt / blend) * 2.0 + EPS,
            "channel " + ch + " jumped faster than a crossfade allows at t=" + t);
        // And it moves monotonically toward the new color.
        double direction = Math.signum(fresh - old);
        assertTrue(
            (current - previous) * direction >= -EPS,
            "channel " + ch + " reversed direction mid-blend at t=" + t);
      }
      prev = now;
    }
  }
}
