package frc.robot.subsystems.LED;

import java.util.Arrays;
import java.util.Comparator;

/**
 * Validated section map of the strip. The map comes from {@link LEDConstants#SECTIONS} until the
 * real wiring is known. Any invalid configuration (empty, out of range, overlapping) falls back to
 * one BALL_PATH section over the whole strip with a console warning naming the problem, so a bad
 * constant can never crash the loop or write outside the buffer.
 */
public final class LEDLayout {
  private final int totalPixels;
  private final LEDSection[] sections;

  private LEDLayout(int totalPixels, LEDSection[] sections) {
    this.totalPixels = totalPixels;
    this.sections = sections;
  }

  /**
   * Validated sections from the constant table, or the whole-strip BALL_PATH fallback (with a
   * System.err warning) on any invalid config.
   */
  public static LEDLayout fromConstants() {
    return of(LEDConstants.LENGTH_PIXELS, LEDConstants.SECTIONS);
  }

  /**
   * Validated layout for an explicit strip length and section list; the injectable sibling of
   * {@link #fromConstants()} used by tests.
   */
  public static LEDLayout of(int totalPixels, LEDSection... sections) {
    String problem = validate(totalPixels, sections);
    if (problem != null) {
      System.err.println(
          "[LEDLayout] Invalid section configuration ("
              + problem
              + "); using one BALL_PATH section over the whole strip");
      int fallbackPixels = Math.max(totalPixels, 1);
      return new LEDLayout(
          fallbackPixels,
          new LEDSection[] {new LEDSection(0, fallbackPixels, false, LEDSection.Role.BALL_PATH)});
    }
    return new LEDLayout(totalPixels, sections.clone());
  }

  /** Null when the configuration is usable, otherwise a short description of the first problem. */
  private static String validate(int totalPixels, LEDSection[] sections) {
    if (sections == null || sections.length == 0) {
      return "no sections configured";
    }
    for (int i = 0; i < sections.length; i++) {
      LEDSection section = sections[i];
      if (section == null) {
        return "section " + i + " is null";
      }
      if (section.startPixel() < 0) {
        return "section " + i + " starts at negative pixel " + section.startPixel();
      }
      if (section.lengthPixels() <= 0) {
        return "section " + i + " has no pixels";
      }
      if (section.startPixel() + section.lengthPixels() > totalPixels) {
        return "section "
            + i
            + " ends at pixel "
            + (section.startPixel() + section.lengthPixels())
            + ", past the strip length "
            + totalPixels;
      }
    }
    LEDSection[] sorted = sections.clone();
    Arrays.sort(sorted, Comparator.comparingInt(LEDSection::startPixel));
    for (int i = 1; i < sorted.length; i++) {
      int previousEnd = sorted[i - 1].startPixel() + sorted[i - 1].lengthPixels();
      if (sorted[i].startPixel() < previousEnd) {
        return "sections overlap at pixel " + sorted[i].startPixel();
      }
    }
    return null;
  }

  /** Validated sections, in configuration order; never empty. */
  public LEDSection[] sections() {
    return sections.clone();
  }

  /** One-line layout summary for the log. */
  public String describe() {
    StringBuilder builder = new StringBuilder();
    builder.append(totalPixels).append(" px, ").append(sections.length).append(" section(s):");
    for (LEDSection section : sections) {
      builder
          .append(" [")
          .append(section.startPixel())
          .append("..")
          .append(section.startPixel() + section.lengthPixels() - 1)
          .append(' ')
          .append(section.role());
      if (section.reversed()) {
        builder.append(" rev");
      }
      builder.append(']');
    }
    return builder.toString();
  }
}
