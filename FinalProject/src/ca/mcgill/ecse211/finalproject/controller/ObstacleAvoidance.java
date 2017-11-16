package ca.mcgill.ecse211.finalproject.controller;

import ca.mcgill.ecse211.finalproject.sensor.UltrasonicController;

/**
 * Handles avoiding of obstacles such as blocks and possibly opponent robot. This is important as if the robot would
 * to get stuck on an object, the odometer would go out of sync and the robot would be completely lost.
 */
public class ObstacleAvoidance implements UltrasonicController{
	
	/**
	 * Constructor for the class ObstacleAvoidance which links parameters to class variables.
     */
	public ObstacleAvoidance() {}

	/**
	 * Method responsible for performing bang-bang navigation to avoid obstacle
	 */
	public void performBangBang(){
		
	}

	/**
	 * Performs any processing of ultrasonic sensor data.
	 *
	 * @param usData ultrasonic sensor reading.
	 */
	@Override
	public void processUSData(float usData) {
		// TODO Auto-generated method stub
		
	}

	/**
	 * Retrieves distance read by ultrasonic sensor.
	 *
	 * @return ultrasonic sensor reading.
	 */
	@Override
	public float readUSData() {
		// TODO Auto-generated method stub
		return 0;
	}

}
