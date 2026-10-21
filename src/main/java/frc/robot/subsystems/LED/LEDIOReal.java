package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * roboRIO PWM output driving the WS2811 COB strip (green data wire → PWM signal pin, strip power
 * from the PDP, grounds tied). Also used in sim, where AddressableLED maps to a harmless HAL sim
 * device.
 */
public class LEDIOReal implements LEDIO {
  private final AddressableLED led;

  public LEDIOReal(int pwmPort, int lengthPixels) {
    led = new AddressableLED(pwmPort);
    led.setLength(lengthPixels);
    led.setData(new AddressableLEDBuffer(lengthPixels)); // start dark
    led.start();
  }

  @Override
  public void writeBuffer(AddressableLEDBuffer buffer) {
    led.setData(buffer);
  }
}
