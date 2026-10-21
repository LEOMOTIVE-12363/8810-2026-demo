package frc.robot.subsystems.LED;

/** Constants for the LED strip subsystem. */
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

  // Effect tuning — placeholder values until the strip is on the robot, adjust freely.
  // Hues are WPILib HSV units (0-180).
  public static final int SOLID_HUE = 22; // gold
  public static final int CHASE_HUE = 160; // blue
  public static final int BREATHE_HUE = 22; // gold
  public static final double CHASE_SPEED_PIXELS_PER_SEC = 20.0;
  public static final int CHASE_TAIL_PIXELS = 6;
  public static final double BREATHE_PERIOD_SEC = 2.0;
  public static final double RAINBOW_FULL_CYCLE_SEC = 3.0;
}
