package ca.mcgill.ecse211.finalproject.sensor;

/**
 * Interface for reading/processing ultrasonic sensor data
 */
public interface UltrasonicController {
		
	/**
	 * Performs any processing of ultrasonic sensor data. 
	 * @param usData ultrasonic sensor reading
	 */
	  public void processUSData(float usData);
	  
	  /**
	   * Retrieves distance read by ultrasonic sensor
	   * @return ultrasonic sensor reading
	   */
	  public float readUSData();
	}
