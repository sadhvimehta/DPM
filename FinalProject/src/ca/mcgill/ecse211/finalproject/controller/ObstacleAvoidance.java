package ca.mcgill.ecse211.finalproject.controller;

import ca.mcgill.ecse211.finalproject.sensor.UltrasonicController;

/**
 * Handles avoiding of obstacles
 */
public class ObstacleAvoidance implements UltrasonicController{
	
	/**
	 * Method responsible for performing bang-bang navigation to avoid obstacle
	 */
	public void performBangBang(){
		
	}
	@Override
	public void processUSData(float usData) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public float readUSData() {
		// TODO Auto-generated method stub
		return 0;
	}

}
