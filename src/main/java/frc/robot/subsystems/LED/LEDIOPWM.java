package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * roboRIO PWM output driving the WS2811 COB strip (green data wire → PWM signal pin, strip power
 * from the PDP, grounds tied). Also used in sim, where AddressableLED maps to a harmless HAL sim
 * device.
 *
 * <p>The incoming buffer holds logical RGB colors; this driver owns the physical color-order
 * conversion (the WS2811 R/G swap) exactly once, for every pattern including the legacy effects.
 * Brightness is already applied by the render layer and is not touched here.
 */
public class LEDIOPWM implements LEDIO {
  private final AddressableLED led;
  private final AddressableLEDBuffer outputBuffer;
  private final boolean swapRedGreen;

  public LEDIOPWM(int pwmPort, int lengthPixels, boolean swapRedGreen) {
    this.swapRedGreen = swapRedGreen;
    led = new AddressableLED(pwmPort);
    led.setLength(lengthPixels);
    outputBuffer = new AddressableLEDBuffer(lengthPixels);
    led.setData(outputBuffer); // start dark
    led.start();
  }

  @Override
  public void writeBuffer(AddressableLEDBuffer buffer) {
    copyInStripColorOrder(buffer, outputBuffer, swapRedGreen);
    led.setData(outputBuffer);
  }

  /**
   * Copies logical-RGB colors into the strip's physical channel order: a straight copy, or a
   * red/green channel swap when the installed WS2811 strip orders its channels GRB.
   */
  static void copyInStripColorOrder(
      AddressableLEDBuffer source, AddressableLEDBuffer destination, boolean swapRedGreen) {
    int length = Math.min(source.getLength(), destination.getLength());
    if (swapRedGreen) {
      for (int i = 0; i < length; i++) {
        destination.setRGB(i, source.getGreen(i), source.getRed(i), source.getBlue(i));
      }
    } else {
      for (int i = 0; i < length; i++) {
        destination.setRGB(i, source.getRed(i), source.getGreen(i), source.getBlue(i));
      }
    }
  }
}
