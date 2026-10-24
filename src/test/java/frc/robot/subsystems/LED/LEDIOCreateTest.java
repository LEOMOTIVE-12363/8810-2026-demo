package frc.robot.subsystems.LED;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants;
import org.junit.jupiter.api.Test;

/**
 * The driver-selection entry must construct exactly one driver per (mode, hardware) pair — the
 * unselected physical device is never opened — and REPLAY must get the empty no-op implementation.
 *
 * <p>The PWM branch is deliberately not constructed here: HAL PWM/DIO allocation is JVM-global
 * under the mocked HAL and never freed, so constructing another AddressableLED from this test could
 * poison unrelated suites depending on class order.
 */
class LEDIOCreateTest {

  @Test
  void defaultCreateSelectsCandle() {
    assertEquals(LEDConstants.Hardware.CANDLE, LEDConstants.HARDWARE);
    LEDIO io = LEDIO.create();
    assertInstanceOf(LEDIOCandle.class, io);
    assertFalse(io instanceof LEDIOPWM, "CANDLE must not open the PWM driver");
  }

  @Test
  void simWithCandleSelectsTheCandleDriver() {
    LEDIO io = LEDIO.create(Constants.Mode.SIM, LEDConstants.Hardware.CANDLE);
    assertInstanceOf(LEDIOCandle.class, io);
    assertFalse(io instanceof LEDIOPWM, "the PWM port must not be opened in CANDLE mode");
  }

  @Test
  void realWithCandleSelectsTheRealCandleDriver() {
    // Constructs a real CANdle handle (no device present on the desktop; the config call just
    // returns a failed status, which the driver reports without throwing).
    LEDIO io = LEDIO.create(Constants.Mode.REAL, LEDConstants.Hardware.CANDLE);
    assertInstanceOf(LEDIOCandle.class, io);
    assertFalse(io instanceof LEDIOPWM, "the PWM port must not be opened in CANDLE mode");
  }

  @Test
  void replayGetsTheEmptyImplementationForEitherHardware() {
    for (LEDConstants.Hardware hardware : LEDConstants.Hardware.values()) {
      LEDIO io = LEDIO.create(Constants.Mode.REPLAY, hardware);
      assertNotEquals(LEDIOPWM.class, io.getClass());
      assertNotEquals(LEDIOCandle.class, io.getClass());
      // The default writeBuffer is a no-op: no hardware output in replay.
      io.writeBuffer(new AddressableLEDBuffer(LEDConstants.LENGTH_PIXELS));
    }
  }
}
