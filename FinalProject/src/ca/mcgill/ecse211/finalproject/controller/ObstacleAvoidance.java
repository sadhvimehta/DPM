package ca.mcgill.ecse211.finalproject.controller;

import ca.mcgill.ecse211.finalproject.sensor.UltrasonicController;

/**
 * Handles avoiding of obstacles
 */
public class ObstacleAvoidance implements UltrasonicController{

	public ObstacleAvoidance() {}

	/**
	 * Method responsible for performing bang-bang navigation to avoid obstacle
	 */
	public void performBangBang(){
		
	}

	/**
	 * Performs any processing of ultrasonic sensor data.
	 *
	 * @param usData ultrasonic sensor reading
	 */
	@Override
	public void processUSData(float usData) {
		// TODO Auto-generated method stub
		
	}

	/**
	 * Retrieves distance read by ultrasonic sensor
	 *
	 * @return ultrasonic sensor reading
	 */
	@Override
	public float readUSData() {
		// TODO Auto-generated method stub
		return 0;
	}

}
