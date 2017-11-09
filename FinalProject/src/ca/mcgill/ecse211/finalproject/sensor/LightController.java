package ca.mcgill.ecse211.finalproject.sensor;

/**
 * Interface for reading/processing light sensor data
 */
public interface LightController {

	/**
	 * Performs any processing of light sensor data.
	 *
	 * @param lsData light intensity reading
	 */
	public void processLSData(float lsData);

	/**
	 * Retrieves intensity read by light sensor
	 *
	 * @return light sensor reading
	 */
	public float readLSData();
}
