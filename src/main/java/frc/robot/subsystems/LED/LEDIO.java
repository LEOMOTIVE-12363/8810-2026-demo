package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * Hardware abstraction for the addressable LED strip. The empty implementation is used in replay so
 * logged runs stay deterministic.
 */
public interface LEDIO {
  /** Pushes the latest buffer to the strip. */
  default void writeBuffer(AddressableLEDBuffer buffer) {}
}
