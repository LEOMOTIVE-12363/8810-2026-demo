package frc.robot.subsystems.LED;

/**
 * Temporal idle effects layered on a {@link LEDTheme} gradient. The theme (colors) and the idle
 * effect (motion) are chosen independently on the dashboard: an effect modulates brightness over
 * time/space — except {@link #COLOR_STEP}, which also re-arranges the gradient into rotating color
 * blocks. All timing/width numbers are preview proposals centralized in {@link LEDConstants} for
 * real-strip tuning.
 */
public enum IdleEffect {
  /** Fixed spatial gradient, no temporal motion at all. */
  STATIC("Static"),
  /** Whole strip breathing in phase between the minimum brightness and full. */
  BREATHE("Breathe"),
  /** Wide brightness wave slowly traveling the strip. */
  SLOW_FLOW("Slow Flow"),
  /** One soft-edge bright band chasing along the strip. */
  CHASE("Chase"),
  /** Wide color blocks rotating through the theme's color order, crossfaded between steps. */
  COLOR_STEP("Color Step");

  private final String dashboardName;

  IdleEffect(String dashboardName) {
    this.dashboardName = dashboardName;
  }

  /** Label shown for this effect on the dashboard chooser. */
  public String getDashboardName() {
    return dashboardName;
  }

  /**
   * Exact dashboard-name match; an unknown or null name returns {@code fallback} (never throws), so
   * a stale saved setting cannot break boot.
   */
  public static IdleEffect fromDashboardName(String name, IdleEffect fallback) {
    if (name != null) {
      for (IdleEffect effect : values()) {
        if (effect.dashboardName.equals(name)) {
          return effect;
        }
      }
    }
    return fallback;
  }
}
