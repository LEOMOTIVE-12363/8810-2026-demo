package frc.robot.subsystems.LED;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.BallFlowTracker.FlowState;
import java.util.function.Function;

/**
 * Ball-flow action layer: while the intake→shooter chain runs, soft brightness bands travel along
 * the BALL_PATH sections over the theme gradient — forward while intaking or feeding the shooter,
 * backward on a full-chain reverse — replacing the idle motion on those pixels only. The bands mean
 * "the mechanism is running in this direction"; they do NOT track the position of individual game
 * pieces.
 *
 * <p>State advances in continuous time injected by the caller (never per-integer-pixel steps): the
 * phase survives target changes, direction reversals slew through zero instead of teleporting, and
 * the whole layer fades in/out over {@link LEDConstants#FLOW_FADE_SEC}. All numbers are preview
 * proposals centralized in {@link LEDConstants} for real-strip tuning.
 */
public final class BallFlowLayer {
  private double phase; // reference band position along the path, in path units; never reset
  private double blend; // 0 = pure idle colors, 1 = full action layer
  private double directionSign = 1.0; // slewed travel direction: +1 intake→shooter, −1 reverse

  public BallFlowLayer() {}

  /**
   * Advances the layer. {@code dtSec} comes from the caller's replay-safe clock; {@code target} is
   * the effective flow state. Idle only fades the layer out — the phase keeps running and the
   * direction is held, so resuming the action continues from where it stopped.
   */
  public void update(double dtSec, FlowState target) {
    double signTarget =
        switch (target) {
          case FORWARD -> 1.0;
          case REVERSE -> -1.0;
          case IDLE -> directionSign; // hold the last direction while idle
        };
    double maxSignStep = 2.0 / LEDConstants.FLOW_REVERSE_SLEW_SEC * dtSec;
    directionSign += MathUtil.clamp(signTarget - directionSign, -maxSignStep, maxSignStep);

    phase += directionSign * (1.0 / LEDConstants.FLOW_TRAVEL_SEC) * dtSec;

    double blendTarget = target == FlowState.IDLE ? 0.0 : 1.0;
    double maxBlendStep = dtSec / LEDConstants.FLOW_FADE_SEC;
    blend += MathUtil.clamp(blendTarget - blend, -maxBlendStep, maxBlendStep);
  }

  /**
   * Blends the moving band envelope (theme-colored) onto the BALL_PATH section pixels, which must
   * already hold their idle colors; DECORATION sections are left untouched. No-op while the layer
   * is fully faded out. Sections are expected to come from a validated {@link LEDLayout}; indexes
   * are clamped to the buffer regardless, so an out-of-range section can never write out of bounds.
   */
  public void apply(
      AddressableLEDBuffer buffer, LEDSection[] sections, Function<Double, Color> themeColor) {
    if (blend <= 0.0) {
      return;
    }
    double halfWidth = LEDConstants.FLOW_BAND_WIDTH_PATH / 2.0;
    double firstCenter = IdleEffects.positiveModulo(phase, LEDConstants.FLOW_BAND_SPACING_PATH);
    for (LEDSection section : sections) {
      if (section.role() != LEDSection.Role.BALL_PATH) {
        continue;
      }
      int start = Math.max(section.startPixel(), 0);
      int end = Math.min(section.startPixel() + section.lengthPixels(), buffer.getLength());
      for (int i = start; i < end; i++) {
        double u = section.uForPixel(i);
        double maxBump = 0.0;
        // Start one spacing below the first wrapped center so bands slide in past u=0 instead of
        // popping into existence at the intake end.
        for (double center = firstCenter - LEDConstants.FLOW_BAND_SPACING_PATH;
            center <= 1.0 + halfWidth;
            center += LEDConstants.FLOW_BAND_SPACING_PATH) {
          maxBump = Math.max(maxBump, IdleEffects.sinSquaredBand(Math.abs(u - center), halfWidth));
        }
        double envelope = LEDConstants.FLOW_FLOOR + (1.0 - LEDConstants.FLOW_FLOOR) * maxBump;
        Color flow = IdleEffects.scale(themeColor.apply(u), envelope);
        Color blended = IdleEffects.lerp(buffer.getLED(i), flow, blend);
        buffer.setRGB(
            i,
            IdleEffects.to255(blended.red),
            IdleEffects.to255(blended.green),
            IdleEffects.to255(blended.blue));
      }
    }
  }

  /** Current band reference position in path units; for logs and tests. */
  public double getPhase() {
    return phase;
  }

  /** Current fade between idle colors (0) and the full action layer (1); for logs and tests. */
  public double getBlend() {
    return blend;
  }

  /** Current slewed travel direction (+1 forward … −1 reverse); for logs and tests. */
  public double getDirectionSign() {
    return directionSign;
  }
}
