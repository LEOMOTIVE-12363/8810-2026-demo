package frc.robot.subsystems.LED;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.BallFlowTracker.FlowState;
import java.util.function.Function;
import org.junit.jupiter.api.Test;

/**
 * The ball-flow layer expresses "the mechanism is running" as theme-colored brightness bands moving
 * along the logical ball path (never per-ball position tracking). Phase must integrate the caller's
 * dt exactly under any frame sizes, direction reversals must slew through zero instead of
 * teleporting the bands to the other end, and the layer only ever touches BALL_PATH pixels.
 */
class BallFlowLayerTest {
  private static final double EPS = 1e-9;
  private static final double DT = 0.02;

  /** All channels well above zero so envelope ratios survive 8-bit rounding in the buffer. */
  private static final Function<Double, Color> TEST_THEME =
      u -> new Color(0.3 + 0.7 * u, 0.5, 1.0 - 0.4 * u);

  private static void run(BallFlowLayer layer, FlowState target, double dt, int steps) {
    for (int i = 0; i < steps; i++) {
      layer.update(dt, target);
    }
  }

  private static double flowSpeed() {
    return 1.0 / LEDConstants.FLOW_TRAVEL_SEC;
  }

  /**
   * Independent re-derivation of the contract's band envelope, used to check apply() per pixel.
   * Bands cover the whole path: centers may sit just below u=0 or above u=1 and still light the end
   * pixels.
   */
  private static double expectedEnvelope(double phase, double u) {
    double spacing = LEDConstants.FLOW_BAND_SPACING_PATH;
    double halfWidth = LEDConstants.FLOW_BAND_WIDTH_PATH / 2.0;
    double wrapped = phase - Math.floor(phase / spacing) * spacing;
    double maxBump = 0.0;
    for (int k = -2; k <= 3; k++) {
      double center = wrapped + k * spacing;
      double distance = Math.min(1.0, Math.abs(u - center) / halfWidth);
      double sine = Math.sin(Math.PI / 2.0 * (1.0 - distance));
      maxBump = Math.max(maxBump, sine * sine);
    }
    return LEDConstants.FLOW_FLOOR + (1.0 - LEDConstants.FLOW_FLOOR) * maxBump;
  }

  /** Steps the layer forward until a band center sits (within one fine step) on pixel `pixel`. */
  private static void alignBandCenterToPixel(BallFlowLayer layer, LEDSection section, int pixel) {
    double u = section.uForPixel(pixel);
    double spacing = LEDConstants.FLOW_BAND_SPACING_PATH;
    double phase = layer.getPhase();
    double target = u + spacing * Math.ceil((phase - u) / spacing + 1e-9);
    if (target <= phase + 1e-6) {
      target += spacing;
    }
    while (layer.getPhase() < target) {
      layer.update(0.001, FlowState.FORWARD);
    }
  }

  @Test
  void blendReachesFullAfterOneFadeTime() {
    BallFlowLayer layer = new BallFlowLayer();
    assertEquals(0.0, layer.getBlend(), EPS, "a fresh layer is invisible");
    run(layer, FlowState.FORWARD, DT, 17); // 0.34 s — just short of the 0.35 s fade
    assertTrue(layer.getBlend() < 1.0, "blend still fading in");
    layer.update(DT, FlowState.FORWARD); // 0.36 s total
    assertEquals(1.0, layer.getBlend(), EPS, "blend clamps at full after the fade time (±dt)");
  }

  @Test
  void phaseAdvancesOnePathPerTravelSecond() {
    BallFlowLayer layer = new BallFlowLayer();
    run(layer, FlowState.FORWARD, DT, 50); // 1.0 s warm-up: the initial direction slew is over
    double phase0 = layer.getPhase();
    run(layer, FlowState.FORWARD, DT, 50); // exactly one more second
    assertEquals(1.0 / LEDConstants.FLOW_TRAVEL_SEC, layer.getPhase() - phase0, EPS);
  }

  @Test
  void variableFrameSizesIntegrateTheSameAsUniform() {
    BallFlowLayer variable = new BallFlowLayer();
    for (int i = 0; i < 50; i++) {
      variable.update(0.01, FlowState.FORWARD);
      variable.update(0.03, FlowState.FORWARD);
    }
    BallFlowLayer uniform = new BallFlowLayer();
    run(uniform, FlowState.FORWARD, DT, 100);

    // Same 2.0 s of mechanism runtime must give the same layer state regardless of frame sizes.
    assertEquals(uniform.getPhase(), variable.getPhase(), 1e-9, "phase");
    assertEquals(uniform.getBlend(), variable.getBlend(), 1e-9, "blend");
    assertEquals(uniform.getDirectionSign(), variable.getDirectionSign(), 1e-9, "direction");
  }

  @Test
  void directionReversalSlewsThroughZeroWithoutTeleporting() {
    BallFlowLayer layer = new BallFlowLayer();
    run(layer, FlowState.FORWARD, DT, 50); // 1.0 s at full forward direction

    double slewDt = 0.005;
    double previousPhase = layer.getPhase();
    double elapsed = 0.0;
    Double crossedAt = null;
    for (int i = 0; i < 100; i++) { // 0.5 s of REVERSE
      layer.update(slewDt, FlowState.REVERSE);
      elapsed += slewDt;
      double phase = layer.getPhase();
      assertTrue(
          Math.abs(phase - previousPhase) <= flowSpeed() * slewDt * 1.5 + 1e-12,
          "phase teleported during reversal at t=" + elapsed);
      previousPhase = phase;
      if (crossedAt == null && layer.getDirectionSign() < 0.0) {
        crossedAt = elapsed;
      }
    }
    assertNotNull(crossedAt, "direction never reversed");
    assertTrue(
        crossedAt <= LEDConstants.FLOW_REVERSE_SLEW_SEC + slewDt + EPS,
        "direction must slew through zero within about one slew time, took " + crossedAt);
  }

  @Test
  void idleHoldNeitherResetsNorStallsThePhase() {
    BallFlowLayer layer = new BallFlowLayer();
    run(layer, FlowState.FORWARD, DT, 25); // 0.5 s forward
    double phaseBeforeIdle = layer.getPhase();
    assertTrue(phaseBeforeIdle > 0.0);

    run(layer, FlowState.IDLE, DT, 10); // 0.2 s hold: direction sign holds, phase keeps moving
    double blendAfterIdle = layer.getBlend();
    run(layer, FlowState.FORWARD, DT, 15); // 0.3 s forward again

    // No reset to zero, no stall during the hold, no rewind on resume — and the layer faded.
    assertEquals(flowSpeed() * 0.5, layer.getPhase() - phaseBeforeIdle, EPS);
    assertEquals(
        1.0 - 0.2 / LEDConstants.FLOW_FADE_SEC,
        blendAfterIdle,
        1e-9,
        "the layer fades out at the contracted rate while idle");
  }

  @Test
  void applyScalesThemeColorByTheBandEnvelopeOnBallPathPixels() {
    LEDSection section = new LEDSection(0, 20, false, LEDSection.Role.BALL_PATH);
    BallFlowLayer layer = new BallFlowLayer();
    run(layer, FlowState.FORWARD, DT, 100); // blend fully in, direction at full speed
    assertEquals(1.0, layer.getBlend(), EPS);
    // Align mid-path (u ~= 0.32) so the band layout is unambiguous whichever way an implementation
    // covers the path ends; the end-coverage question is checked separately by eye on the strip.
    alignBandCenterToPixel(layer, section, 6);

    AddressableLEDBuffer buffer = new AddressableLEDBuffer(20);
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, 12, 34, 56); // standing "idle" content; blend == 1 replaces it entirely
    }
    layer.apply(buffer, new LEDSection[] {section}, TEST_THEME);

    double minRatio = Double.POSITIVE_INFINITY;
    double maxRatio = Double.NEGATIVE_INFINITY;
    for (int i = 0; i < buffer.getLength(); i++) {
      double u = section.uForPixel(i);
      Color themeColor = TEST_THEME.apply(u);
      Color actual = buffer.getLED(i);
      double ratioR = actual.red / themeColor.red;
      double ratioG = actual.green / themeColor.green;
      double ratioB = actual.blue / themeColor.blue;
      // One scalar envelope multiplies the theme color: all channels fade together, and the scalar
      // is the contract's band envelope for this pixel.
      assertEquals(ratioR, ratioG, 0.04, "channels not scaled by one envelope, pixel " + i);
      assertEquals(ratioR, ratioB, 0.04, "channels not scaled by one envelope, pixel " + i);
      assertEquals(expectedEnvelope(layer.getPhase(), u), ratioR, 0.04, "envelope, pixel " + i);
      minRatio = Math.min(minRatio, ratioR);
      maxRatio = Math.max(maxRatio, ratioR);
    }
    assertTrue(minRatio >= 0.15, "darkest pixel must still respect the 18% floor");
    assertTrue(maxRatio >= 0.95, "pixel under the band center is at full theme brightness");
  }

  @Test
  void applyLeavesDecorationPixelsUntouched() {
    LEDSection ballPath = new LEDSection(0, 10, false, LEDSection.Role.BALL_PATH);
    LEDSection decoration = new LEDSection(10, 10, false, LEDSection.Role.DECORATION);
    BallFlowLayer layer = new BallFlowLayer();
    run(layer, FlowState.FORWARD, DT, 100); // blend == 1

    AddressableLEDBuffer buffer = new AddressableLEDBuffer(20);
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, 5, 6, 7);
    }
    layer.apply(buffer, new LEDSection[] {ballPath, decoration}, TEST_THEME);

    for (int i = 10; i < 20; i++) {
      // Raw 8-bit channel reads: getLED() ceil-quantizes to 1/4096 on the way back, ints are exact.
      assertEquals(5, buffer.getRed(i), "decoration pixel " + i + " red");
      assertEquals(6, buffer.getGreen(i), "decoration pixel " + i + " green");
      assertEquals(7, buffer.getBlue(i), "decoration pixel " + i + " blue");
    }
  }

  @Test
  void applyStaysInBoundsOnATinyStrip() {
    LEDSection section = new LEDSection(0, 3, false, LEDSection.Role.BALL_PATH);
    BallFlowLayer layer = new BallFlowLayer();
    run(layer, FlowState.FORWARD, DT, 40);
    AddressableLEDBuffer buffer = new AddressableLEDBuffer(3);
    // Must not throw or write past the last pixel.
    layer.apply(buffer, new LEDSection[] {section}, TEST_THEME);
    for (int i = 0; i < buffer.getLength(); i++) {
      Color c = buffer.getLED(i);
      assertTrue(c.red >= 0.0 && c.red <= 1.0, "pixel " + i + " red in range");
    }
  }
}
