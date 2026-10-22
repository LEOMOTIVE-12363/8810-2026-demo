package frc.robot.subsystems.LED;

/**
 * Constants for the LED strip subsystem — the single tuning surface for the Day of the Dead themes,
 * idle effects and the ball-flow action layer. All animation numbers are preview proposals from the
 * design preview, to be tuned on the real strip once installed; distances are in normalized
 * ball-path units (0 = intake entrance, 1 = shooter top), times in seconds.
 */
public final class LEDConstants {
  private LEDConstants() {}

  // TODO(strip install): confirm which roboRIO PWM port (silkscreen 0-9) the green data wire is
  // plugged into. Placeholder until then.
  public static final int PWM_PORT = 0;

  // The strip is a 12 V WS2811 COB strip: one IC per 5 cm segment => 20 addressable pixels per
  // meter (the advertised 720 LEDs/m counts chips, not controllable pixels).
  public static final double PIXELS_PER_METER = 20.0;

  // TODO(strip install): replace with the measured installed length. Placeholder assumes 3.0 m
  // => 60 pixels; pixels beyond the physical strip are ignored by the strip, so erring high only
  // costs bandwidth.
  public static final double LENGTH_METERS = 3.0;
  public static final int LENGTH_PIXELS = (int) Math.round(LENGTH_METERS * PIXELS_PER_METER);

  // Legacy effect tuning — placeholder values until the strip is on the robot, adjust freely.
  // Hues are WPILib HSV units (0-180).
  public static final int SOLID_HUE = 22; // gold
  public static final int CHASE_HUE = 160; // blue
  public static final int BREATHE_HUE = 22; // gold
  public static final double CHASE_SPEED_PIXELS_PER_SEC = 20.0;
  public static final int CHASE_TAIL_PIXELS = 6;
  public static final double BREATHE_PERIOD_SEC = 2.0;
  public static final double RAINBOW_FULL_CYCLE_SEC = 3.0;

  // TODO(strip color order): WS2811 strips vary in R/G channel order; flip to true if theme colors
  // look red/green-swapped on the real strip. Applies to the theme patterns only.
  public static final boolean SWAP_RED_GREEN_FOR_WS2811 = false;

  // Theme switching (operator action): RGB crossfade between the old and new gradient.
  public static final double THEME_TRANSITION_SEC = 0.6;

  // Idle effects (motion on top of the theme gradient, chosen independently of the theme).
  public static final double IDLE_BREATHE_PERIOD_SEC = 4.8;
  public static final double IDLE_MIN_BRIGHTNESS = 0.35; // shared by BREATHE and SLOW_FLOW
  public static final double IDLE_SLOW_FLOW_PERIOD_SEC = 8.0;
  public static final double IDLE_CHASE_PERIOD_SEC = 3.6;
  public static final double IDLE_CHASE_BAND_WIDTH_PATH = 0.25; // total band width
  public static final double IDLE_CHASE_FLOOR = 0.18;
  public static final int IDLE_COLOR_STEP_BLOCK_COUNT = 4;
  public static final double IDLE_COLOR_STEP_PERIOD_SEC = 1.2; // one color-order rotation step
  public static final double IDLE_COLOR_STEP_BLEND_SEC = 0.18; // RGB crossfade between steps

  // Ball-flow action layer (moving bands on BALL_PATH sections while the mechanism runs).
  public static final double FLOW_BAND_WIDTH_PATH = 0.30; // total width of one band
  public static final double FLOW_BAND_SPACING_PATH = 0.65; // center-to-center band spacing
  public static final double FLOW_FLOOR = 0.18; // brightness between bands
  public static final double FLOW_TRAVEL_SEC = 1.8; // one band crossing the full path
  public static final double FLOW_FADE_SEC = 0.35; // layer blend in/out
  public static final double FLOW_REVERSE_SLEW_SEC = 0.12; // direction reversal through zero

  // TODO(strip wiring): placeholder section map — one BALL_PATH section over the whole strip, so
  // the flow layer gives direction indication only until the real wiring is known. Then replace
  // with one entry per physical run: absolute start pixel, length in pixels, reversed flag (set
  // when that run is installed intake-at-the-end, so u=0 stays the intake entrance), and role
  // (BALL_PATH for intake/shooter runs, DECORATION for purely cosmetic runs).
  public static final LEDSection[] SECTIONS = {
    new LEDSection(0, LENGTH_PIXELS, false, LEDSection.Role.BALL_PATH)
  };
}
