package ca.mcgill.ecse211.finalproject.controller;

import ca.mcgill.ecse211.finalproject.odometry.Odometer;
import ca.mcgill.ecse211.finalproject.sensor.LightController;
import ca.mcgill.ecse211.finalproject.sensor.UltrasonicController;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * Detects block color using light sensor readings.
 *
 * In the competition, this class will take care of first finding blocks and secondly finding the right block which
 * represents the opponents flag.
 *
 */
public class BlockDetection implements UltrasonicController, LightController{

	/**
	 * Odometer of the robot
	 */
	private Odometer odometer;
	/**
	 * Left motor of the robot
	 */
	private EV3LargeRegulatedMotor leftMotor;
	/**
	 * Right motor of the robot
	 */
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

	/**
	 * Constructor of the class BlockDetection, which links the parameters to the class variables.
	 */
	public BlockDetection(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, SampleProvider csSensor, float[] csData){
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.csSensor = csSensor;
		this.csData = csData;
	}

	/**
	 * Main method of this class which will contain logic to go about finding the opponents flag. This also represents
	 * one of the states that controller will be in along the competition.
	 */
	private void findFlag() {

	}
	
	/**
	 * Determines color of detected block using intensityBuffer values. This will therefore help to determine if block
	 * found is indeed the flag of the opposing team.
	 * @return number representing color of detected block
	 */
	private int determineColor(){
		return 0;
	}
	
	/**
	 * Provides the intensity of block from various distances to populate intensity buffer. This will help get an idea
	 * of where the blocks possibly are.
	 */
	private void populateIntensityBuffer(){
		
	}

	/**
	 * Performs any processing of light sensor data.
	 *
	 * @param lsData light intensity reading
	 */
	@Override
	public void processLSData(float lsData) {
		// TODO Auto-generated method stub
		
	}

	/**
	 * Retrieves intensity read by light sensor
	 *
	 * @return light sensor reading
	 */
	@Override
	public float readLSData() {
		// TODO Auto-generated method stub
		return 0;
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


	@Override
	public float readUSData() {
		// TODO Auto-generated method stub
		return 0;
	}

}
