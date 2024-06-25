package frc.robot.subsystems.transport;

public interface TransportIO {
  /**
   * Sets the transport speeds. Speeds are how fast the edge wheels will move at, in meters per second.
   * This is effectively the speed that the note moves.  
   * @param speedMetersPerSecond The speed that the intake and transport wheels will spin, in meters per second.
   */
  public default void setTransportSpeed(double speedMetersPerSecond) {}
}