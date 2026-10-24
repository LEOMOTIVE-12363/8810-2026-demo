package frc.robot.subsystems.LED;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.subsystems.LED.LEDConstants.Hardware;
import org.junit.jupiter.api.Test;

/**
 * Pins the "LED Driver" dashboard label: it must name the selected scheme and echo the wiring knobs
 * it depends on, so a stale label can never hide a mismatched deploy.
 */
class LEDHardwareLabelTest {

  @Test
  void pwmLabelsTheSchemeAndPort() {
    assertEquals("PWM (port 0)", LED.hardwareLabel(Hardware.PWM, 0, 0, "rio"));
    assertEquals("PWM (port 7)", LED.hardwareLabel(Hardware.PWM, 7, 3, "canivore"));
  }

  @Test
  void candleLabelsTheSchemeCanIdAndBus() {
    assertEquals("CANdle (CAN 0, bus rio)", LED.hardwareLabel(Hardware.CANDLE, 0, 0, "rio"));
    assertEquals(
        "CANdle (CAN 2, bus canivore)", LED.hardwareLabel(Hardware.CANDLE, 9, 2, "canivore"));
  }

  @Test
  void configuredLabelMatchesCandleSelection() {
    assertEquals(
        "CANdle (CAN 0, bus rio)",
        LED.hardwareLabel(
            LEDConstants.HARDWARE,
            LEDConstants.PWM_PORT,
            LEDConstants.CANDLE_CAN_ID,
            LEDConstants.CANDLE_CAN_BUS));
  }
}
