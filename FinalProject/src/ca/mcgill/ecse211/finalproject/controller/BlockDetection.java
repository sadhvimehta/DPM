package ca.mcgill.ecse211.finalproject.controller;

import ca.mcgill.ecse211.finalproject.odometry.Odometer;
import ca.mcgill.ecse211.finalproject.sensor.LightController;
import ca.mcgill.ecse211.finalproject.sensor.UltrasonicController;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * Detects block color using light sensor readings
 *
 */
public class BlockDetection implements UltrasonicController, LightController{
	
	private Odometer odo;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	/**
	 * Sample provider
	 */
	private SampleProvider csSensor;
	/**
	 * Array containing data obtained from light sensor
	 */
    private float[] csData;
    /**
     * Buffer that contains intensity readings from range of distances
     */
    private float[] intensityBuffer;
	
	public BlockDetection(Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, SampleProvider csSensor, float[] csData){
		this.odo = odo;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.csSensor = csSensor;
		this.csData = csData;
	}
	
	/**
	 * Determines color of detected block using intensityBuffer values
	 * @return number representing color of detected block
	 */
	private int determineColor(){
		return 0;
	}
	
	/**
	 * Provides the intensity of block from various distances to populate intensity buffer
	 */
	private void populateIntensityBuffer(){
		
	}

	@Override
	public void processLSData(float lsData) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public float readLSData() {
		// TODO Auto-generated method stub
		return 0;
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
