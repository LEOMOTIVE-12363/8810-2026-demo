package frc.robot.subsystems.LED;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.LED.LEDSection.Role;
import org.junit.jupiter.api.Test;

/**
 * The section table describes physical wiring that is not known yet, so any invalid table must
 * degrade to one whole-strip BALL_PATH section (with a warning) instead of ever writing out of
 * buffer bounds. These tests pin that fallback plus the section-local logical-u mapping — the
 * reversed flag is what lets two physically mirrored ball-path strips show the same flow direction.
 */
class LEDLayoutTest {
  private static final int TOTAL_PIXELS = 20;
  private static final double EPS = 1e-12;

  private static void assertWholeStripFallback(LEDLayout layout) {
    assertEquals(1, layout.sections().length, "invalid config must fall back to one section");
    LEDSection fallback = layout.sections()[0];
    assertEquals(0, fallback.startPixel());
    assertEquals(TOTAL_PIXELS, fallback.lengthPixels());
    assertFalse(fallback.reversed());
    assertEquals(Role.BALL_PATH, fallback.role());
  }

  @Test
  void validTwoSectionLayoutIsKeptAsConfigured() {
    LEDSection first = new LEDSection(0, 10, false, Role.BALL_PATH);
    LEDSection second = new LEDSection(10, 10, true, Role.DECORATION);
    LEDLayout layout = LEDLayout.of(TOTAL_PIXELS, first, second);

    assertEquals(2, layout.sections().length);
    LEDSection kept = layout.sections()[1];
    assertEquals(10, kept.startPixel());
    assertEquals(10, kept.lengthPixels());
    assertTrue(kept.reversed());
    assertEquals(Role.DECORATION, kept.role());
    assertNotNull(layout.describe());
    assertFalse(layout.describe().isEmpty());
  }

  @Test
  void mirroredBallPathTwinsShareTheSameLogicalFlow() {
    LEDSection forward = new LEDSection(0, 10, false, Role.BALL_PATH);
    LEDSection mirrored = new LEDSection(10, 10, true, Role.BALL_PATH);

    // The reversed flag flips the mapping so the two mirrored strips agree on u: the first pixel of
    // one section is the same logical point as the last pixel of its twin.
    double prevForward = -1.0;
    double prevMirrored = Double.POSITIVE_INFINITY;
    for (int i = 0; i < 10; i++) {
      double uForward = forward.uForPixel(i);
      double uMirrored = mirrored.uForPixel(10 + i);
      assertEquals(uForward, mirrored.uForPixel(19 - i), EPS, "twin mirror at pixel " + i);
      assertTrue(uForward >= 0.0 && uForward <= 1.0, "u in [0,1] at pixel " + i);
      assertTrue(uForward > prevForward, "u increases along the section at pixel " + i);
      assertTrue(uMirrored < prevMirrored, "reversed u decreases along the strip at pixel " + i);
      prevForward = uForward;
      prevMirrored = uMirrored;
    }
  }

  @Test
  void singlePixelSectionStillProducesAValidCoordinate() {
    LEDSection single = new LEDSection(4, 1, false, Role.BALL_PATH);
    double u = single.uForPixel(4);
    assertTrue(Double.isFinite(u) && u >= 0.0 && u <= 1.0, "single-pixel u must not be NaN");
  }

  @Test
  void emptySectionTableFallsBackToWholeStrip() {
    assertWholeStripFallback(LEDLayout.of(TOTAL_PIXELS));
  }

  @Test
  void overlappingSectionsFallBackToWholeStrip() {
    LEDSection[] overlapping = {
      new LEDSection(0, 10, false, Role.BALL_PATH), new LEDSection(5, 10, false, Role.DECORATION)
    };
    assertWholeStripFallback(LEDLayout.of(TOTAL_PIXELS, overlapping));
  }

  @Test
  void outOfRangeSectionsFallBackToWholeStrip() {
    // Ends at pixel 25 on a 20-pixel strip.
    LEDSection[] outOfRange = {new LEDSection(15, 10, false, Role.BALL_PATH)};
    assertWholeStripFallback(LEDLayout.of(TOTAL_PIXELS, outOfRange));
  }

  @Test
  void zeroLengthSectionFallsBackToWholeStrip() {
    LEDSection[] zeroLength = {new LEDSection(7, 0, false, Role.BALL_PATH)};
    assertWholeStripFallback(LEDLayout.of(TOTAL_PIXELS, zeroLength));
  }

  @Test
  void constantsPlaceholderIsOneWholeStripBallPathSection() {
    // Until real wiring is known, the constants table is a single direction-indication placeholder
    // covering the full strip.
    LEDLayout layout = LEDLayout.fromConstants();
    assertEquals(1, layout.sections().length);
    LEDSection only = layout.sections()[0];
    assertEquals(0, only.startPixel());
    assertEquals(LEDConstants.LENGTH_PIXELS, only.lengthPixels());
    assertFalse(only.reversed());
    assertEquals(Role.BALL_PATH, only.role());
  }
}
