package ca.mcgill.ecse211.finalproject.sensor;

/** Interface for reading/processing light sensor data */
public interface LightController {
  /**
   * Retrieves intensity read by light sensor
   *
   * @return light sensor reading
   */
  public float readLSData();
}
