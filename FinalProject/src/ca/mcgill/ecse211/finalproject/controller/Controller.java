package ca.mcgill.ecse211.finalproject.controller;

import ca.mcgill.ecse211.finalproject.sensor.UltrasonicController;

public class Controller extends Thread implements UltrasonicController{

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
