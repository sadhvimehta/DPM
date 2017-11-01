package ca.mcgill.ecse211.finalproject.controller;

import ca.mcgill.ecse211.finalproject.sensor.LightController;
import ca.mcgill.ecse211.finalproject.sensor.UltrasonicController;

public class BlockDetection implements UltrasonicController, LightController{

	@Override
	public void processLSData(float lsData) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public int readLSDistance() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public void processUSData(float usData) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public int readLSData() {
		// TODO Auto-generated method stub
		return 0;
	}

}
