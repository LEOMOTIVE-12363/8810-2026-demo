package frc.robot.subsystems.LED;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import org.junit.jupiter.api.Test;

/**
 * The PWM driver owns the logical→physical color-order conversion exactly once. These tests pin the
 * copy/swap math (the strip output itself is HAL-side and not readable back, so the conversion is
 * tested at this seam).
 */
class LEDIOPWMTest {

  @Test
  void copyIsIdentityWithoutSwap() {
    AddressableLEDBuffer source = filled(5);
    AddressableLEDBuffer destination = new AddressableLEDBuffer(5);

    LEDIOPWM.copyInStripColorOrder(source, destination, false);

    for (int i = 0; i < 5; i++) {
      assertEquals(source.getRed(i), destination.getRed(i), "red at " + i);
      assertEquals(source.getGreen(i), destination.getGreen(i), "green at " + i);
      assertEquals(source.getBlue(i), destination.getBlue(i), "blue at " + i);
    }
  }

  @Test
  void copySwapsRedAndGreenOnlyWhenFlagged() {
    AddressableLEDBuffer source = filled(5);
    AddressableLEDBuffer destination = new AddressableLEDBuffer(5);

    LEDIOPWM.copyInStripColorOrder(source, destination, true);

    for (int i = 0; i < 5; i++) {
      assertEquals(source.getGreen(i), destination.getRed(i), "red slot gets green at " + i);
      assertEquals(source.getRed(i), destination.getGreen(i), "green slot gets red at " + i);
      assertEquals(source.getBlue(i), destination.getBlue(i), "blue untouched at " + i);
    }
  }

  @Test
  void copyHandlesShorterDestinationBuffers() {
    AddressableLEDBuffer source = filled(8);
    AddressableLEDBuffer destination = new AddressableLEDBuffer(3);

    // Must not throw or read/write past either buffer.
    LEDIOPWM.copyInStripColorOrder(source, destination, true);

    assertEquals(source.getGreen(2), destination.getRed(2));
  }

  private static AddressableLEDBuffer filled(int pixels) {
    AddressableLEDBuffer buffer = new AddressableLEDBuffer(pixels);
    for (int i = 0; i < pixels; i++) {
      buffer.setRGB(i, 10 + i, 20 + i, 30 + i);
    }
    return buffer;
  }
}
