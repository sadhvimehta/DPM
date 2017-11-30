package ca.mcgill.ecse211.finalproject.sensor;

/**
 * Interface for reading/processing ultrasonic sensor data
 */
public interface UltrasonicController {
	/**
	 * Retrieves distance read by ultrasonic sensor
	 *
	 * @return ultrasonic sensor reading
	 */
	public float readUSData();
}

